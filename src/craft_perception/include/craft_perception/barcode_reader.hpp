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

// Where a barcode may be, found without knowing where it is placed (user 2026-11-14: the barcode's place is not
// fixed, only that it is in the upper cam's view). On a work_width copy: |  |gx| - |gy|  | (bars give a strong
// gradient in one direction only), threshold, close along the bars' run, keep blobs that are elongated and
// filled. Rects are in full-image pixels, padded for the quiet zone, best first (gradient x area).
struct BarcodeLocatorConfig {
  int work_width = 640;
  float min_gradient = 120.0f;  // Scharr |gx|-|gy| after a 5x5 mean
  int min_area = 120;           // work px (bag 2026-11-11: the symbol ~40x12 px at 640x360)
  int min_side = 4;             // work px
  double min_aspect = 1.5, max_aspect = 12.0;
  double min_fill = 0.5;        // contour area / its min-area rect
  int max_regions = 6;
  double pad = 0.3;             // of the rect's width on each side (quiet zone), same height fraction x2
  bool valid() const;
};
std::vector<cv::Rect> find_barcode_regions(const cv::Mat &image, const BarcodeLocatorConfig &config = {});
// decode_code128() on each region (upscaled x2 when narrower than 400 px), texts deduplicated.
std::vector<std::string> decode_code128_regions(const cv::Mat &image, const std::vector<cv::Rect> &regions);

// Fallback when no symbol decodes (user 2026-11-14): the mission has only four codes, so instead of reading the bars
// one by one, compare the bar profile with the four Code 128 patterns. Per region: turn the bars upright from the
// dominant gradient, average rows along the bars at x4 into 1D profiles, and score each code by normalized
// correlation over module widths 0.6..4.5 px and both directions.
// Bags 2026-11-14 (ZXing 0 / 2 of 423 / 33 of 656): per-frame vote right 95% / 77% / 93% (python prototype).
struct KnownCodeMatch {
  std::string text;   // best code
  double score = 0;   // its normalized correlation, 0..1
  double margin = 0;  // over the second best
};
std::optional<KnownCodeMatch> match_known_code128(const cv::Mat &image, const std::vector<cv::Rect> &regions,
                                                  const std::vector<std::string> &codes);

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
  // Pattern-match vote of one frame; confirmed once `votes` frames picked one code and it holds >= share of all.
  bool vote(const std::string &text, int votes, double share);
  const std::map<std::string, int> &votes() const { return votes_; }
  bool confirmed_by_vote() const { return by_vote_; }

private:
  int confirm_;
  std::map<std::string, int> votes_;
  bool by_vote_ = false;
  std::map<std::string, int> counts_;
  std::optional<std::string> text_;
};
}
