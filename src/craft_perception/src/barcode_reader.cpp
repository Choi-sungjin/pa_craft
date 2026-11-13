#include "craft_perception/barcode_reader.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#ifdef CRAFT_WITH_ZXING
#include <ZXing/BitMatrix.h>
#include <ZXing/MultiFormatWriter.h>
#include <ZXing/ReadBarcode.h>
#endif
#include <cmath>

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

bool BarcodeLocatorConfig::valid() const {
  return work_width >= 64 && min_gradient > 0 && min_area > 0 && min_side > 0 && min_aspect >= 1.0 &&
         max_aspect > min_aspect && min_fill > 0 && min_fill <= 1.0 && max_regions > 0 && pad >= 0 && pad <= 2.0;
}

std::vector<cv::Rect> find_barcode_regions(const cv::Mat &image, const BarcodeLocatorConfig &config) {
  std::vector<cv::Rect> out;
  if (image.empty() || image.depth() != CV_8U || !config.valid()) return out;
  cv::Mat grey;
  if (image.channels() == 3) cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 1) grey = image;
  else return out;
  const double s = std::min(1.0, static_cast<double>(config.work_width) / grey.cols);
  cv::Mat small;
  if (s < 1.0) cv::resize(grey, small, cv::Size(), s, s, cv::INTER_AREA);
  else small = grey;
  cv::Mat gx, gy, diff, mask;
  cv::Scharr(small, gx, CV_32F, 1, 0);
  cv::Scharr(small, gy, CV_32F, 0, 1);
  cv::absdiff(cv::abs(gx), cv::abs(gy), diff);
  cv::blur(diff, diff, cv::Size(5, 5));
  cv::compare(diff, config.min_gradient, mask, cv::CMP_GT);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 3)));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<std::pair<double, cv::Rect>> scored;
  for (const auto &c : contours) {
    const auto r = cv::minAreaRect(c);
    const double w = r.size.width, h = r.size.height;
    if (std::min(w, h) < config.min_side || w * h < config.min_area) continue;
    const double aspect = std::max(w, h) / std::min(w, h);
    if (aspect < config.min_aspect || aspect > config.max_aspect) continue;
    if (cv::contourArea(c) / (w * h) < config.min_fill) continue;
    const cv::Rect box = cv::boundingRect(c);
    scored.emplace_back(cv::mean(diff(box))[0] * box.area(), box);
  }
  std::sort(scored.begin(), scored.end(), [](const auto &a, const auto &b) {return a.first > b.first;});
  const cv::Rect frame(0, 0, grey.cols, grey.rows);
  for (const auto &[score, box] : scored) {
    if (static_cast<int>(out.size()) >= config.max_regions) break;
    const double x = box.x / s, y = box.y / s, w = box.width / s, h = box.height / s;
    const double px = config.pad * std::max(w, h) + 8.0;  // quiet zone either way the bars run
    const cv::Rect padded(cv::Point(static_cast<int>(x - px), static_cast<int>(y - px)),
                          cv::Point(static_cast<int>(x + w + px + 1), static_cast<int>(y + h + px + 1)));
    const cv::Rect clipped = padded & frame;
    if (clipped.area() > 0) out.push_back(clipped);
  }
  return out;
}

std::vector<std::string> decode_code128_regions(const cv::Mat &image, const std::vector<cv::Rect> &regions) {
  std::vector<std::string> out;
  const cv::Rect frame(0, 0, image.cols, image.rows);
  for (const auto &region : regions) {
    const cv::Rect r = region & frame;
    if (r.area() <= 0) continue;
    cv::Mat crop = image(r);
    if (crop.cols < 400) cv::resize(crop, crop, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
    for (auto &text : decode_code128(crop)) {
      if (std::find(out.begin(), out.end(), text) == out.end()) out.push_back(std::move(text));
    }
  }
  return out;
}

namespace {
// Code 128 modules of `text` (1 = bar), no quiet zone.
std::vector<float> code128_modules(const std::string &text) {
  std::vector<float> m;
#ifdef CRAFT_WITH_ZXING
  try {
    const auto bits = ZXing::MultiFormatWriter(ZXing::BarcodeFormat::Code128).setMargin(0).encode(text, 0, 0);
    for (int x = 0; x < bits.width(); ++x) m.push_back(bits.get(x, 0) ? 1.0f : 0.0f);
  } catch (...) {m.clear();}
#else
  (void)text;
#endif
  return m;
}

constexpr int kUp = 4;     // profile upsampling
constexpr int kQuiet = 12;  // light modules rendered on either side

// 1D darkness profiles across the bars of one region (bars turned upright), standardized; empty when no bars.
std::vector<cv::Mat> bar_profiles(const cv::Mat &grey, const cv::Rect &region) {
  std::vector<cv::Mat> out;
  const cv::Rect r = region & cv::Rect(0, 0, grey.cols, grey.rows);
  if (r.width < 8 || r.height < 8) return out;
  cv::Mat c; grey(r).convertTo(c, CV_32F);
  cv::Mat gx, gy; cv::Sobel(c, gx, CV_32F, 1, 0, 3); cv::Sobel(c, gy, CV_32F, 0, 1, 3);
  const double sxy = cv::sum(gx.mul(gy))[0], sxx = cv::sum(gx.mul(gx))[0], syy = cv::sum(gy.mul(gy))[0];
  const double angle = 0.5 * std::atan2(2 * sxy, sxx - syy);  // gradient direction = across the bars
  cv::Mat c4; cv::resize(c, c4, cv::Size(), kUp, kUp, cv::INTER_CUBIC);
  const cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f(c4.cols / 2.0f, c4.rows / 2.0f), angle * 180.0 / CV_PI, 1.0);
  cv::Mat u; cv::warpAffine(c4, u, rot, c4.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  cv::Mat dx = cv::abs(u.colRange(1, u.cols) - u.colRange(0, u.cols - 1));
  cv::Mat energy; cv::reduce(dx, energy, 1, cv::REDUCE_AVG);
  double emax; cv::minMaxLoc(energy, nullptr, &emax);
  int lo = -1, hi = -1;
  for (int y = 0; y < energy.rows; ++y) if (energy.at<float>(y) > 0.5 * emax) {if (lo < 0) lo = y; hi = y;}
  if (lo < 0 || hi - lo < 4 * kUp) return out;
  for (const auto &[f0, f1] : {std::pair{0.25, 0.75}, {0.1, 0.4}, {0.6, 0.9}}) {
    const int a = lo + static_cast<int>((hi - lo) * f0), b = lo + static_cast<int>((hi - lo) * f1);
    if (b <= a) continue;
    cv::Mat p; cv::reduce(u.rowRange(a, b), p, 0, cv::REDUCE_AVG);
    cv::Scalar mean, sd; cv::meanStdDev(p, mean, sd);
    if (sd[0] < 1e-3) continue;
    out.push_back((mean[0] - p) / sd[0]);  // dark bars positive
  }
  return out;
}

// Best normalized correlation of the rendered code against the profile, module widths [w_lo, w_hi] px (at kUp).
double code_score(const cv::Mat &p, const std::vector<float> &modules, double w_lo, double w_hi) {
  double best = -1;
  for (double w = w_lo; w <= w_hi; w += 0.1 * kUp) {
    const int n = static_cast<int>((modules.size() + 2 * kQuiet) * w);
    if (n < 16 || n > p.cols) continue;
    cv::Mat t(1, n, CV_32F);
    for (int x = 0; x < n; ++x) {
      const int k = static_cast<int>((x + 0.5) / w) - kQuiet;
      t.at<float>(x) = k >= 0 && k < static_cast<int>(modules.size()) ? modules[k] : 0.0f;
    }
    cv::GaussianBlur(t, t, cv::Size(0, 0), std::max(0.4 * w, 1.0), 0);
    for (const bool flip : {false, true}) {
      cv::Mat q = t;
      if (flip) cv::flip(t, q, 1);
      cv::Mat r; cv::matchTemplate(p, q, r, cv::TM_CCOEFF_NORMED);
      double m; cv::minMaxLoc(r, nullptr, &m);
      best = std::max(best, m);
    }
  }
  return best;
}
}

std::optional<KnownCodeMatch> match_known_code128(const cv::Mat &image, const std::vector<cv::Rect> &regions,
                                                  const std::vector<std::string> &codes) {
  if (image.empty() || image.depth() != CV_8U || codes.size() < 2) return std::nullopt;
  std::vector<std::vector<float>> patterns;
  for (const auto &code : codes) {
    patterns.push_back(code128_modules(code));
    if (patterns.back().empty() || patterns.back().size() != patterns.front().size()) return std::nullopt;
  }
  cv::Mat grey;
  if (image.channels() == 3) cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
  else grey = image;
  std::optional<KnownCodeMatch> best;
  for (const auto &region : regions) {
    for (const auto &p : bar_profiles(grey, region)) {
      // Module width: from 0.6 px up to what fits the profile (bar span estimates were off on bent labels).
      const double w_hi = std::min(4.5 * kUp, static_cast<double>(p.cols) / (patterns.front().size() + 2 * kQuiet));
      if (w_hi < 0.6 * kUp) continue;
      std::vector<double> scores;
      for (const auto &m : patterns) scores.push_back(code_score(p, m, 0.6 * kUp, w_hi));
      std::vector<std::size_t> order(scores.size());
      for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
      std::sort(order.begin(), order.end(), [&](auto a, auto b) {return scores[a] > scores[b];});
      KnownCodeMatch m{codes[order[0]], scores[order[0]], scores[order[0]] - scores[order[1]]};
      if (!best || m.score > best->score) best = m;
    }
  }
  return best;
}

bool BarcodeReader::vote(const std::string &text, int votes, double share) {
  if (text_) return false;
  const int n = ++votes_[text];
  int total = 0;
  for (const auto &[t, k] : votes_) total += k;
  if (n >= votes && n >= share * total) {
    text_ = text;
    by_vote_ = true;
    return true;
  }
  return false;
}

void BarcodeReader::reset() {
  counts_.clear();
  votes_.clear();
  by_vote_ = false;
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
