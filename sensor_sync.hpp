#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

enum class SensorType { CAMERA, IMU };

struct SensorData {
  SensorType type;
  std::uint32_t sequence;
  long long timestamp_us;
};

struct SyncResult {
  SensorData camera;
  std::optional<SensorData> imu;
  long long delta_us = 0;
};

namespace detail {
inline bool packetOrder(const SensorData& lhs, const SensorData& rhs) {
  if (lhs.timestamp_us != rhs.timestamp_us) {
    return lhs.timestamp_us < rhs.timestamp_us;
  }
  return lhs.sequence < rhs.sequence;
}

inline unsigned long long absoluteDelta(long long lhs, long long rhs) {
  if (lhs >= rhs) {
    return static_cast<unsigned long long>(lhs) - static_cast<unsigned long long>(rhs);
  }
  return static_cast<unsigned long long>(rhs) - static_cast<unsigned long long>(lhs);
}

inline bool betterMatch(const SensorData& lhs, const SensorData& rhs, long long camera_ts) {
  const unsigned long long lhs_delta = absoluteDelta(lhs.timestamp_us, camera_ts);
  const unsigned long long rhs_delta = absoluteDelta(rhs.timestamp_us, camera_ts);
  if (lhs_delta != rhs_delta) {
    return lhs_delta < rhs_delta;
  }
  if (lhs.timestamp_us != rhs.timestamp_us) {
    return lhs.timestamp_us < rhs.timestamp_us; 
  }
  return lhs.sequence < rhs.sequence; 
}

} 
inline std::vector<SyncResult> synchronize(
    std::vector<SensorData> cameras,
    std::vector<SensorData> imus,
    long long max_delta_us) {
  std::sort(cameras.begin(), cameras.end(), detail::packetOrder);
  std::sort(imus.begin(), imus.end(), detail::packetOrder);

  if (max_delta_us < 0) {
    throw std::invalid_argument("max_delta_us must be greater than or equal to 0");
  }
  const auto threshold = static_cast<unsigned long long>(max_delta_us);
  const std::size_t kNoImu = imus.size();

  std::vector<SyncResult> results;
  results.reserve(cameras.size());
  std::size_t right = 0;
  for (const SensorData& camera : cameras) {
    while (right < imus.size() && imus[right].timestamp_us <= camera.timestamp_us) {
      ++right; 
    }

    std::size_t best_index = kNoImu;
    if (right > 0) {
      std::size_t candidate = right - 1;
      while (candidate > 0 &&
             imus[candidate - 1].timestamp_us == imus[candidate].timestamp_us) {
        --candidate;
      }
      best_index = candidate;
    }
    if (right < imus.size()) {。
      if (best_index == kNoImu ||
          detail::betterMatch(imus[right], imus[best_index], camera.timestamp_us)) {
        best_index = right;
      }
    }

    SyncResult result;
    result.camera = camera;

    if (best_index == kNoImu) {
      results.push_back(std::move(result));  
      continue;
    }

    const unsigned long long delta =
        detail::absoluteDelta(imus[best_index].timestamp_us, camera.timestamp_us);
    if (delta > threshold) {
      results.push_back(std::move(result));  
    }

    result.imu = imus[best_index];
    result.delta_us = static_cast<long long>(delta);
    results.push_back(std::move(result));
  }

  return results;
}
