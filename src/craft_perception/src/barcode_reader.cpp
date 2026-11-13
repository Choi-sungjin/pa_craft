#include "craft_perception/barcode_reader.hpp"
#include <opencv2/imgproc.hpp>
#ifdef CRAFT_WITH_ZXING
#include <ZXing/ReadBarcode.h>
#endif

namespace craft_perception {
namespace {
std::string trimmed(const std::string &s) {
  const auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
}

bool barcode_decoder_available() {
#ifdef CRAFT_WITH_ZXING
  return true;
#else
  return false;
#endif
}

std::vector<std::string> decode_code128(const cv::Mat &image) {
  std::vector<std::string> out;
#ifdef CRAFT_WITH_ZXING
  if (image.empty() || image.depth() != CV_8U) return out;
  cv::Mat grey;
  if (image.channels() == 3) cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 1) grey = image;
  else return out;
  if (!grey.isContinuous()) grey = grey.clone();
  const ZXing::ImageView view(grey.data, grey.cols, grey.rows, ZXing::ImageFormat::Lum);
  ZXing::ReaderOptions options;
  options.setFormats(ZXing::BarcodeFormat::Code128);
  for (const auto &result : ZXing::ReadBarcodes(view, options)) {
    if (result.isValid()) out.push_back(trimmed(result.text()));
  }
#else
  (void)image;
#endif
  return out;
}

void BarcodeReader::reset() {
  counts_.clear();
  text_.reset();
}

bool BarcodeReader::update(const std::vector<std::string> &texts) {
  if (text_) return false;
  for (const auto &text : texts) {
    if (++counts_[text] >= confirm_) {
      text_ = text;
      return true;
    }
  }
  return false;
}

std::optional<SortOrder> BarcodeReader::order() const {
  return text_ ? parse_barcode(*text_) : std::nullopt;
}
}
