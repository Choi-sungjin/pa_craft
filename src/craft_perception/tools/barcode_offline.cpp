// Offline check of the mission barcode reader on recorded frames.
//   barcode_offline frame1.jpg frame2.jpg ...   (frames in time order, e.g. dumped from an upper-cam bag)
// Same path as perception_master (perception.yaml uppercam.barcode defaults): the pattern vote on the candidate
// regions, no decoder. --decode: ZXing on the whole frame and the regions first (the vote only when it reads
// nothing); --full: ZXing on the whole frame only; --regions: with --decode, ZXing on the regions only.
// No symbol decoded on a frame: the four mission codes are pattern-matched on the regions (--no-match: off) and
// voted (min score 0.6, margin 0.03, 15 votes, 70 % share = perception.yaml uppercam.barcode.match).
// Prints every frame where a Code 128 symbol was decoded, the time per frame, then the confirmed code and sort order.
#include "craft_perception/barcode_reader.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>
#include <string>

using namespace craft_perception;

int main(int argc, char **argv) {
  if (!barcode_decoder_available()) {
    std::fprintf(stderr, "built without zxing-cpp (libzxing-dev)\n");
    return 2;
  }
  BarcodeReader reader;
  int frames = 0, decoded = 0, confirmed_at = -1, located = 0;
  bool full = false;
  double ms_sum = 0, ms_max = 0;
  const BarcodeLocatorConfig locator;
  int full_every = 1;
  bool match = true, decode = false;  // perception.yaml uppercam.barcode.decode false: the vote alone
  int matched = 0;
  double match_ms = 0;
  const std::vector<std::string> codes{"P2-01", "P2-02", "P2-03", "P2-04"};  // perception.yaml uppercam.barcode.full_frame_every
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--full")) {full = true; continue;}
    if (!std::strcmp(argv[i], "--regions")) {full_every = 0; continue;}
    if (!std::strcmp(argv[i], "--no-match")) {match = false; continue;}
    if (!std::strcmp(argv[i], "--decode")) {decode = true; continue;}
    const cv::Mat image = cv::imread(argv[i], cv::IMREAD_COLOR);
    if (image.empty()) continue;
    ++frames;
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::string> texts;
    if (full) {  // ZXing, whole frame
      texts = decode_code128(image);
    } else {
      const auto regions = find_barcode_regions(image, locator);
      located += !regions.empty();
      if (decode) {
        if (full_every > 0 && (frames - 1) % full_every == 0) texts = decode_code128(image);
        for (auto &t : decode_code128_regions(image, regions)) {
          if (std::find(texts.begin(), texts.end(), t) == texts.end()) texts.push_back(std::move(t));
        }
      }
      if (texts.empty() && match && !reader.confirmed()) {
        const auto m0 = std::chrono::steady_clock::now();
        const std::vector<cv::Rect> top(regions.begin(), regions.begin() + std::min<std::size_t>(3, regions.size()));
        const auto m = match_known_code128(image, top, codes);
        match_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m0).count();
        if (m && m->score >= 0.6 && m->margin >= 0.03) {
          ++matched;
          if (reader.vote(m->text, 15, 0.7)) confirmed_at = frames;
        }
      }
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ms_sum += ms; ms_max = std::max(ms_max, ms);
    if (!texts.empty()) {
      ++decoded;
      std::printf("frame %d (%s):", frames, argv[i]);
      for (const auto &t : texts) std::printf(" '%s'", t.c_str());
      std::printf("\n");
    }
    if (reader.update(texts)) confirmed_at = frames;
  }
  std::printf("frames=%d decoded=%d (%.0f%%) %s, %.1f ms/frame mean, %.1f max\n", frames, decoded,
    frames ? 100.0 * decoded / frames : 0.0, full ? "whole frame" : (std::to_string(located) + " with regions").c_str(),
    frames ? ms_sum / frames : 0.0, ms_max);
  for (const auto &[text, n] : reader.reads()) std::printf("  read '%s' x%d\n", text.c_str(), n);
  if (match) {
    std::printf("pattern match: %d votes, %.1f ms/frame when run:", matched, matched ? match_ms / std::max(1, frames - decoded) : 0.0);
    for (const auto &[text, n] : reader.votes()) std::printf(" %s x%d", text.c_str(), n);
    std::printf("\n");
  }
  if (!reader.confirmed()) {
    std::printf("result: NOT CONFIRMED\n");
    return 1;
  }
  const auto order = reader.order();
  std::printf("result: '%s' confirmed at frame %d%s -> %s\n", reader.text()->c_str(), confirmed_at,
    reader.confirmed_by_vote() ? " (pattern vote)" : "",
    !order ? "UNREGISTERED (mission error)" : *order == SortOrder::kAscending ? "ascending" : "descending");
  return order ? 0 : 1;
}
