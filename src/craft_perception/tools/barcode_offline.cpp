// Offline check of the mission barcode reader on recorded frames.
//   barcode_offline frame1.jpg frame2.jpg ...   (frames in time order, e.g. dumped from an upper-cam bag)
// Prints every frame where a Code 128 symbol was decoded, then the confirmed code and sort order.
#include "craft_perception/barcode_reader.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <map>

using namespace craft_perception;

int main(int argc, char **argv) {
  if (!barcode_decoder_available()) {
    std::fprintf(stderr, "built without zxing-cpp (libzxing-dev)\n");
    return 2;
  }
  BarcodeReader reader;
  int frames = 0, decoded = 0, confirmed_at = -1;
  for (int i = 1; i < argc; ++i) {
    const cv::Mat image = cv::imread(argv[i], cv::IMREAD_COLOR);
    if (image.empty()) continue;
    ++frames;
    const auto texts = decode_code128(image);
    if (!texts.empty()) {
      ++decoded;
      std::printf("frame %d (%s):", frames, argv[i]);
      for (const auto &t : texts) std::printf(" '%s'", t.c_str());
      std::printf("\n");
    }
    if (reader.update(texts)) confirmed_at = frames;
  }
  std::printf("frames=%d decoded=%d (%.0f%%)\n", frames, decoded, frames ? 100.0 * decoded / frames : 0.0);
  for (const auto &[text, n] : reader.reads()) std::printf("  read '%s' x%d\n", text.c_str(), n);
  if (!reader.confirmed()) {
    std::printf("result: NOT CONFIRMED\n");
    return 1;
  }
  const auto order = reader.order();
  std::printf("result: '%s' confirmed at frame %d -> %s\n", reader.text()->c_str(), confirmed_at,
    !order ? "UNREGISTERED (mission error)" : *order == SortOrder::kAscending ? "ascending" : "descending");
  return order ? 0 : 1;
}
