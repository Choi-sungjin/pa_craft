#include "craft_perception/shared_frame.hpp"
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace craft_perception {
SharedFrameReader::SharedFrameReader(std::string name, bool depth)
  : name_(std::move(name)), depth_(depth) {
  if (name_.size() < 2 || name_[0] != '/' || name_.find('/', 1) != std::string::npos) {
    throw std::invalid_argument("shared memory name must be /name");
  }
}
SharedFrameReader::~SharedFrameReader() { close_mapping(); }

bool SharedFrameReader::is_valid_metadata(const std::vector<uint64_t> &meta, bool depth) {
  if (meta.size() != (depth ? 6u : 5u) || meta[1] == 0 || meta[2] == 0 ||
      meta[1] > 8192 || meta[2] > 8192 || meta[4] > uint64_t(INT64_MAX) ||
      (depth && (meta[5] == 0 || meta[5] > 1000000000))) return false;
  const uint64_t slot_size = meta[1] * meta[2] * (depth ? 5 : 3) + sizeof(uint64_t);
  return meta[0] % slot_size == 0 && meta[0] / slot_size < 4;
}

void SharedFrameReader::close_mapping() {
  if (mapping_) munmap(const_cast<uint8_t *>(mapping_), mapped_size_);
  if (fd_ >= 0) close(fd_);
  mapping_ = nullptr; mapped_size_ = 0; fd_ = -1;
}

std::shared_ptr<const SharedFrame> SharedFrameReader::snapshot(const std::vector<uint64_t> &meta) {
  if (!is_valid_metadata(meta, depth_)) return {};
  if (cached_ && meta == cached_meta_) { ++cache_hits_; return cached_; }
  // An inode check also detects producer unlink/recreate without needing a new
  // shm_open/mmap on every frame. Producer resizes under the same flock.
  struct stat current{};
  if (stat(("/dev/shm" + name_).c_str(), &current) != 0) return {};
  if (fd_ >= 0 && (device_ != uint64_t(current.st_dev) || inode_ != uint64_t(current.st_ino))) close_mapping();
  if (fd_ < 0) {
    fd_ = shm_open(name_.c_str(), O_RDONLY, 0);
    if (fd_ < 0) return {};
  }
  if (flock(fd_, LOCK_SH | LOCK_NB) != 0) return {};
  struct Unlock { int fd; ~Unlock() { flock(fd, LOCK_UN); } } unlock{fd_};
  struct stat st{};
  const uint64_t pixels = meta[1] * meta[2], color_bytes = pixels * 3;
  const uint64_t payload_bytes = pixels * (depth_ ? 5 : 3);
  if (fstat(fd_, &st) != 0 || st.st_size < 0 ||
      uint64_t(st.st_size) < meta[0] + payload_bytes + sizeof(uint64_t)) return {};
  device_ = st.st_dev; inode_ = st.st_ino;
  if (!mapping_ || mapped_size_ != uint64_t(st.st_size)) {
    if (mapping_) munmap(const_cast<uint8_t *>(mapping_), mapped_size_);
    mapping_ = nullptr; mapped_size_ = 0;
    void *mapped = mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd_, 0);
    if (mapped == MAP_FAILED) return {};
    mapping_ = static_cast<const uint8_t *>(mapped); mapped_size_ = st.st_size;
  }
  const auto *source = mapping_ + meta[0];
  uint64_t sequence;
  std::memcpy(&sequence, source + payload_bytes, sizeof(sequence));
  if (sequence != meta[3]) return {};
  // One contiguous copy, only on demand. Pad the destination if an odd pixel
  // count would otherwise misalign the uint16 depth view.
  const int padding = depth_ ? color_bytes % alignof(uint16_t) : 0;
  cv::Mat storage(1, int(payload_bytes) + padding, CV_8UC1);
  std::memcpy(storage.data + padding, source, payload_bytes);
  auto result = std::make_shared<SharedFrame>();
  result->bgr = storage.colRange(padding, padding + int(color_bytes)).reshape(3, int(meta[2]));
  if (depth_) {
    result->depth.raw = cv::Mat(int(meta[2]), int(meta[1]), CV_16UC1, storage.data + padding + color_bytes);
    result->depth.unit_m = meta[5] * 1e-9;
    result->depth.owner = storage;
  }
  result->sequence = sequence; result->stamp_ns = meta[4];
  cached_meta_ = meta; cached_ = result;
  ++snapshots_; bytes_copied_ += payload_bytes;
  return result;
}
}  // namespace craft_perception
