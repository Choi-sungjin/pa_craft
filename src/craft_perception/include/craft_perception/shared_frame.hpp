#pragma once
#include "craft_perception/perception_types.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace craft_perception {
struct SharedFrame {
  cv::Mat bgr;
  DepthImage depth;
  uint64_t sequence = 0, stamp_ns = 0;
};

// Master serializes each camera's calls. Read-only mmap is reused; a requested
// frame is copied once under a short flock and shared by every consuming module.
// Inference never holds the producer lock. Same metadata returns the same owner.
class SharedFrameReader {
public:
  SharedFrameReader(std::string name, bool depth);
  ~SharedFrameReader();
  SharedFrameReader(const SharedFrameReader &) = delete;
  SharedFrameReader &operator=(const SharedFrameReader &) = delete;
  static bool is_valid_metadata(const std::vector<uint64_t> &meta, bool depth);
  std::shared_ptr<const SharedFrame> snapshot(const std::vector<uint64_t> &meta);
  uint64_t snapshots() const { return snapshots_; }
  uint64_t bytes_copied() const { return bytes_copied_; }
  uint64_t cache_hits() const { return cache_hits_; }
private:
  void close_mapping();
  std::string name_;
  bool depth_;
  int fd_ = -1;
  const uint8_t *mapping_ = nullptr;
  size_t mapped_size_ = 0;
  uint64_t device_ = 0, inode_ = 0;
  uint64_t snapshots_ = 0, bytes_copied_ = 0, cache_hits_ = 0;
  std::vector<uint64_t> cached_meta_;
  std::shared_ptr<const SharedFrame> cached_;
};
}  // namespace craft_perception
