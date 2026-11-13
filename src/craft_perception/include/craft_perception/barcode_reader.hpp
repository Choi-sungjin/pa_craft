#pragma once
#include "craft_perception/traffic_light.hpp"
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
// Mission barcode (Code 128 at the start zone). Port of the perception-v2 package's
// scripts/barcode_reader.py: decode every frame, confirm once the same text was read in
// `confirm` frames (a single misread never wins), then map it to the sort order.

// true when built with zxing-cpp (libzxing-dev); without it decode_code128() finds nothing.
bool barcode_decoder_available();
// Texts of all Code 128 symbols in a BGR or grey image.
std::vector<std::string> decode_code128(const cv::Mat &image);

class BarcodeReader {
public:
  explicit BarcodeReader(int confirm = 3) : confirm_(confirm) {}
  void reset();
  // texts: decode_code128() of one frame. true when a code was just confirmed.
  bool update(const std::vector<std::string> &texts);
  bool confirmed() const { return text_.has_value(); }
  const std::optional<std::string> &text() const { return text_; }
  // Sort order of the confirmed code; nullopt while pending, or for an unregistered code
  // (confirmed() && !order() = mission error: stop safely, no guessing).
  std::optional<SortOrder> order() const;
  const std::map<std::string, int> &reads() const { return counts_; }

private:
  int confirm_;
  std::map<std::string, int> counts_;
  std::optional<std::string> text_;
};
}
