#include "pick_policy.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <tuple>

namespace puck::policy {

namespace {
const std::string N = R"(([+-]?\d+(?:\.\d+)?))";
}

bool Candidate::operator==(const Candidate &o) const {
  return cls == o.cls && score == o.score && ahead == o.ahead && lateral == o.lateral && posture == o.posture &&
         height == o.height && distance == o.distance;
}

nlohmann::ordered_json Candidate::record() const {
  return {{"cls", cls}, {"score", score}, {"ahead", ahead}, {"lateral", lateral}, {"posture", posture},
          {"height", height ? nlohmann::ordered_json(*height) : nlohmann::ordered_json(nullptr)}, {"distance", distance}};
}

std::vector<std::string> splitlines(const std::string &text) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start < text.size()) {
    // str.splitlines() also recognizes vertical tab, form feed, file/group/record
    // separators and Unicode line separators. Detector output is UTF-8.
    size_t end = start, length = 0;
    for (; end < text.size(); ++end) {
      const auto c = static_cast<unsigned char>(text[end]);
      if (c == '\r' || c == '\n' || c == '\v' || c == '\f' || (c >= 0x1c && c <= 0x1e)) {
        length = c == '\r' && end + 1 < text.size() && text[end + 1] == '\n' ? 2 : 1;
        break;
      }
      if (text.compare(end, 2, "\xc2\x85") == 0) { length = 2; break; }
      if (text.compare(end, 3, "\xe2\x80\xa8") == 0 || text.compare(end, 3, "\xe2\x80\xa9") == 0) {
        length = 3;
        break;
      }
    }
    if (!length) {out.push_back(text.substr(start)); break;}
    out.push_back(text.substr(start, end - start));
    start = end + length;
  }
  return out;
}

std::vector<Candidate> candidates(const std::string &text, double ahead_bias, double lateral_bias, double face_x,
                                  double laser_y, double min_score) {
  static const std::regex head("PUCK (\\S+) " + N + " ahead " + N + " lateral " + N);
  static const std::regex state(R"( posture (standing|lying|unknown)\b)");
  static const std::regex height_re(" height " + N);
  std::vector<Candidate> found;
  for (const auto &line : splitlines(text)) {
    std::smatch m;
    if (!std::regex_search(line, m, head, std::regex_constants::match_continuous)) continue;
    Candidate c;
    c.cls = m[1];
    c.score = std::stod(m[2]);
    c.ahead = std::stod(m[3]) - ahead_bias;
    c.lateral = std::stod(m[4]) - lateral_bias;
    static const std::vector<std::string> classes{"red_cylinder", "green_cylinder", "red_prism", "green_prism"};
    if (c.score < min_score || std::find(classes.begin(), classes.end(), c.cls) == classes.end()) continue;
    // Survey belongs to the docked 205 mm box, not other detections in the room.
    if (!(0 <= c.ahead && c.ahead <= 205) || std::abs(c.lateral) > 102.5) continue;
    std::smatch s, h;
    const bool has_state = std::regex_search(line, s, state);
    const bool cyl = c.cls.size() >= 8 && c.cls.compare(c.cls.size() - 8, 8, "cylinder") == 0;
    c.posture = cyl ? "standing" : has_state ? std::string(s[1]) : "unknown";
    if (std::regex_search(line, h, height_re)) c.height = std::stod(h[1]);
    c.distance = std::hypot(face_x + c.ahead, laser_y + c.lateral);
    found.push_back(c);
  }
  return found;
}

Candidate select(const std::vector<Candidate> &found, std::optional<int> expected) {
  if (expected && static_cast<int>(found.size()) != *expected)
    throw ValueError("expected " + std::to_string(*expected) + " puck(s), detected " + std::to_string(found.size()) +
                     "; rescan before choosing");
  if (found.empty()) throw ValueError("no confidently detected puck on the box");
  if (found.size() > 2)
    throw ValueError("expected at most two pucks on this box, detected " + std::to_string(found.size()));
  for (const auto &p : found)
    if (p.posture == "unknown") throw ValueError("prism posture unknown: cannot enforce standing-first priority; rescan");
  auto key = [](const Candidate &p) {return std::make_tuple(p.posture != "standing", p.distance, -p.score, p.cls, p.lateral);};
  return *std::min_element(found.begin(), found.end(), [&](const auto &a, const auto &b) {return key(a) < key(b);});
}

}  // namespace puck::policy
