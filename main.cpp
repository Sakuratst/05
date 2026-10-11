#include <atomic>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sensor_sync.hpp"
#include "thread_safe_queue.hpp"

namespace {

namespace fs = std::filesystem;
enum ExitCode {
  kExitOk = 0,
  kExitUsage = 1,   
  kExitConfig = 2,  
  kExitData = 3,    
  kExitOutput = 4, 
  kExitThread = 5, 
};

class AppError : public std::runtime_error {
 public:
  AppError(std::string message, int exit_code)
      : std::runtime_error(std::move(message)), exit_code_(exit_code) {}

  int exit_code() const noexcept { return exit_code_; }

 private:
  int exit_code_;
};

std::string trim(const std::string& text) {
  std::size_t begin = 0;
  while (begin < text.size() &&
         std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
    ++begin;
  }
  std::size_t end = text.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
    --end;
  }
  return text.substr(begin, end - begin);
}

bool parseUnsigned(const std::string& text, unsigned long long& out) {
  if (text.empty()) {
    return false;
  }
  unsigned long long value = 0;
  for (char ch : text) {
    if (ch < '0' || ch > '9') {
      return false;
    }
    const unsigned long long digit = static_cast<unsigned long long>(ch - '0');
    constexpr unsigned long long kMax = 0xFFFFFFFFFFFFFFFFULL;
    if (value > (kMax - digit) / 10ULL) {
      return false;  // 溢出
    }
    value = value * 10ULL + digit;
  }
  out = value;
  return true;
}

void stripCarriageReturn(std::string& line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
}

bool parsePacket(const std::string& line, SensorType type, SensorData& out) {
  std::vector<std::string> fields;
  std::size_t pos = 0;
  while (true) {
    const std::size_t next = line.find_first_of(" \t", pos);
    if (next == std::string::npos) {
      fields.push_back(line.substr(pos));
      break;
    }
    fields.push_back(line.substr(pos, next - pos));
    pos = line.find_first_not_of(" \t", next);
    if (pos == std::string::npos) {
      break;  
    }
  }

  if (fields.size() != 2) {
    return false;
  }

  unsigned long long sequence = 0;
  unsigned long long timestamp = 0;
  if (!parseUnsigned(fields[0], sequence) || !parseUnsigned(fields[1], timestamp)) {
    return false;
  }
  if (sequence > 0xFFFFFFFFULL || timestamp > 0x7FFFFFFFFFFFFFFFULL) {
    return false;
  }

  out.type = type;
  out.sequence = static_cast<std::uint32_t>(sequence);
  out.timestamp_us = static_cast<long long>(timestamp);
  return true;
}

fs::path resolvePath(const fs::path& base_dir, const std::string& path_text) {
  fs::path path(path_text);
  if (path.is_absolute()) {
    return path;
  }
#if defined(_WIN32)
  const std::string& text = path_text;
  if (text.size() >= 3 && std::isalpha(static_cast<unsigned char>(text[0])) != 0 &&
      text[1] == ':' && (text[2] == '\\' || text[2] == '/')) {
    return path;
  }
#endif
  return base_dir / path;
}

std::string describeSystemError(int error_number) {
  return std::error_code(error_number, std::system_category()).message();
}

struct Config {
  std::size_t queue_capacity = 0;
  long long max_delta_us = 0;
  fs::path camera_file;
  fs::path imu_file;
  fs::path output_file;
};

Config loadConfig(const fs::path& config_path) {
  std::ifstream input(config_path);
  if (!input) {
    throw AppError("cannot open configuration file '" + config_path.string() + "': " +
                       describeSystemError(errno),
                   kExitConfig);
  }

  
  const std::vector<std::string> required_keys = {"queue_capacity", "max_delta_us",
                                                  "camera_file", "imu_file", "output_file"};
  std::unordered_map<std::string, std::string> values;
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(input, line)) {
    ++line_number;
    stripCarriageReturn(line);
    const std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    const std::size_t equals = trimmed.find('=');
    if (equals == std::string::npos) {
      throw AppError("configuration error at line " + std::to_string(line_number) +
                         ": expected 'key=value', got '" + trimmed + "'",
                     kExitConfig);
    }

    const std::string key = trim(trimmed.substr(0, equals));
    const std::string value = trim(trimmed.substr(equals + 1));
    if (key.empty()) {
      throw AppError("configuration error at line " + std::to_string(line_number) +
                         ": empty key",
                     kExitConfig);
    }

    bool known = false;
    for (const std::string& required : required_keys) {
      if (required == key) {
        known = true;
        break;
      }
    }
    if (!known) {
      throw AppError("configuration error at line " + std::to_string(line_number) +
                         ": unknown key '" + key + "'",
                     kExitConfig);
    }
    if (!values.emplace(key, value).second) {
      throw AppError("configuration error at line " + std::to_string(line_number) +
                         ": duplicate key '" + key + "'",
                     kExitConfig);
    }
  }

  for (const std::string& required : required_keys) {
    if (values.find(required) == values.end()) {
      throw AppError("configuration error: missing required key '" + required + "'", kExitConfig);
    }
  }

  Config config;

  unsigned long long capacity = 0;
  if (!parseUnsigned(values["queue_capacity"], capacity) || capacity == 0) {
    throw AppError("configuration error: queue_capacity must be a positive integer, got '" +
                       values["queue_capacity"] + "'",
                   kExitConfig);
  }
  if (capacity > 1000000ULL) {
    throw AppError("configuration error: queue_capacity is unreasonably large (max 1000000), got '" +
                       values["queue_capacity"] + "'",
                   kExitConfig);
  }
  config.queue_capacity = static_cast<std::size_t>(capacity);

  unsigned long long max_delta = 0;
  if (!parseUnsigned(values["max_delta_us"], max_delta) ||
      max_delta > 0x7FFFFFFFFFFFFFFFULL) {
    throw AppError("configuration error: max_delta_us must be a non-negative integer, got '" +
                       values["max_delta_us"] + "'",
                   kExitConfig);
  }
  config.max_delta_us = static_cast<long long>(max_delta);

  const fs::path base_dir = config_path.has_parent_path() ? config_path.parent_path() : fs::path(".");
  const std::string path_keys[3] = {"camera_file", "imu_file", "output_file"};
  fs::path* path_targets[3] = {&config.camera_file, &config.imu_file, &config.output_file};
  for (int i = 0; i < 3; ++i) {
    if (values[path_keys[i]].empty()) {
      throw AppError("configuration error: '" + path_keys[i] + "' must not be empty", kExitConfig);
    }
    *path_targets[i] = resolvePath(base_dir, values[path_keys[i]]);
  }

  return config;
}


 public:
  void report(std::string message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!message_.has_value()) {
      message_ = std::move(message);
    }
  }

  bool hasError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_.has_value();
  }

  std::string message() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_.value_or(std::string());
  }

 private:
  mutable std::mutex mutex_;
  std::optional<std::string> message_;
};

bool produce(const fs::path& file, SensorType type, ThreadSafeQueue<SensorData>& queue,
             ErrorReporter& errors) {
  std::ifstream input(file);
  if (!input) {
    errors.report("cannot open data file '" + file.string() + "': " + describeSystemError(errno));
    return false;
  }

  const std::string label = (type == SensorType::CAMERA) ? "camera" : "imu";
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    stripCarriageReturn(line);
    if (line.empty()) {
  
      errors.report(label + " file '" + file.string() + "' line " +
                    std::to_string(line_number) + ": empty line is not a valid packet");
      return false;
    }

    SensorData packet{};
    if (!parsePacket(line, type, packet)) {
      errors.report(label + " file '" + file.string() + "' line " +
                    std::to_string(line_number) + ": malformed packet '" + trim(line) + "'");
      return false;
    }

    
    if (!queue.push(packet)) {
      return false;
    }
  }

  if (input.bad()) {
    errors.report("read error on data file '" + file.string() + "'");
    return false;
  }
  return true;
}

void consume(ThreadSafeQueue<SensorData>& queue, std::vector<SensorData>& cameras,
             std::vector<SensorData>& imus, ErrorReporter& errors) {
  try {
    SensorData packet{};
    while (queue.pop(packet)) {
      if (packet.type == SensorType::CAMERA) {
        cameras.push_back(packet);
      } else {
        imus.push_back(packet);
      }
    }
  } catch (const std::exception& ex) {
  
    errors.report(std::string("consumer thread failed: ") + ex.what());
    queue.close();
  }
}


void writeResults(const fs::path& output_file, const std::vector<SyncResult>& results) {
  std::ofstream output(output_file, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw AppError("cannot open output file '" + output_file.string() + "' for writing: " +
                       describeSystemError(errno),
                   kExitOutput);
  }

  for (const SyncResult& result : results) {
    output << "CAMERA " << result.camera.sequence;
    if (result.imu.has_value()) {
      output << " IMU " << result.imu->sequence << " DELTA " << result.delta_us;
    } else {
      output << " UNMATCHED";
    }
    output << '\n';
  }
  output.flush();

  if (!output) {
    throw AppError("failed to write output file '" + output_file.string() + "'",
                   kExitOutput);
  }
}

class ThreadGuard {
 public:
  explicit ThreadGuard(std::vector<std::thread>& threads) : threads_(threads) {}

  ~ThreadGuard() {
    for (std::thread& thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

  ThreadGuard(const ThreadGuard&) = delete;
  ThreadGuard& operator=(const ThreadGuard&) = delete;

 private:
  std::vector<std::thread>& threads_;
};

void printUsage(const char* program) {
  std::cerr << "usage: " << program << " <config_file>\n";
}

}  

int main(int argc, char* argv[]) {
  if (argc == 2) {
    const std::string argument = argv[1];
    if (argument == "-h" || argument == "--help") {
      printUsage(argv[0]);
      return kExitOk;
    }
  }
  if (argc != 2) {
    printUsage(argv[0]);
    return kExitUsage;
  }

  try {
    const Config config = loadConfig(fs::path(argv[1]));

    ThreadSafeQueue<SensorData> queue(config.queue_capacity);
    ErrorReporter errors;
    std::vector<SensorData> cameras;
    std::vector<SensorData> imus;

    std::atomic<int> producers_left{2};
    std::mutex close_mutex;

    std::vector<std::thread> threads;
    ThreadGuard guard(threads);

    const auto onProducerDone = [&queue, &producers_left, &close_mutex]() {
      std::lock_guard<std::mutex> lock(close_mutex);
      if (producers_left.fetch_sub(1) == 1) {
        queue.close();  
      }
    };

    try {
      threads.emplace_back([&] {
        const bool ok = produce(config.camera_file, SensorType::CAMERA, queue, errors);
        onProducerDone();
        (void)ok;
      });
      threads.emplace_back([&] {
        const bool ok = produce(config.imu_file, SensorType::IMU, queue, errors);
        onProducerDone();
        (void)ok;
      });
      threads.emplace_back(
          [&] { consume(queue, cameras, imus, errors); });
    } catch (const std::exception& ex) {
      queue.close();
      throw AppError(std::string("failed to start worker threads: ") + ex.what(), kExitThread);
    }

    for (std::thread& thread : threads) {
      thread.join();
    }

    if (errors.hasError()) {
      throw AppError(errors.message(), kExitData);
    }

    const std::vector<SyncResult> results =
        synchronize(std::move(cameras), std::move(imus), config.max_delta_us);
    writeResults(config.output_file, results);

    return kExitOk;
  } catch (const AppError& ex) {
    std::cerr << "sensor_sync: " << ex.what() << '\n';
    return ex.exit_code();
  } catch (const std::exception& ex) {
    std::cerr << "sensor_sync: unexpected error: " << ex.what() << '\n';
    return kExitConfig;
  }
}
