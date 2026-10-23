#include "craft_planning/path_planner.hpp"

#include <png.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace craft_planning
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfinity = 1e15;
class RouteUnavailable : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
struct Point {double x, y;};
double squared(Point p) {return p.x * p.x + p.y * p.y;}
Point operator-(Point a, Point b) {return {a.x - b.x, a.y - b.y};}
Point lerp(Point a, Point b, double t) {return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};}

struct Grid
{
  int width = 0, height = 0;
  double res = 0, ox = 0, oy = 0, yaw = 0;
  std::vector<uint8_t> occupied, blocked;
  std::vector<double> clearance;
  bool contains(int x, int y) const {return x >= 0 && y >= 0 && x < width && y < height;}
  int index(int x, int y) const {return y * width + x;}
  Point center(int index) const {return {(index % width + 0.5) * res, (index / width + 0.5) * res};}
  Point local(const geometry_msgs::msg::Point & p) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw), x = p.x - ox, y = p.y - oy;
    return {c * x + s * y, -s * x + c * y};
  }
  int cell(Point p) const
  {
    if (!(p.x >= 0 && p.y >= 0 && p.x < width * res && p.y < height * res)) {return -1;}
    return index(static_cast<int>(p.x / res), static_cast<int>(p.y / res));
  }
};

void require(bool valid, const std::string & error)
{
  if (!valid) {throw std::invalid_argument(error);}
}

void allocate(Grid & g, double width, double height, const PathPlannerConfig & config)
{
  require(std::isfinite(width) && std::isfinite(height) && width >= 1 && height >= 1 &&
    width <= config.max_cells && height <= config.max_cells &&
    width * height <= config.max_cells, "Map dimensions exceed max_cells or are empty");
  g.width = static_cast<int>(width); g.height = static_cast<int>(height);
  g.occupied.assign(static_cast<std::size_t>(g.width) * g.height, 0);
}

// Felzenszwalb/Huttenlocher squared Euclidean distance transform, one axis at a time.
// Finite sentinel avoids inf-inf in intersection calculations on empty map rows.
void distance_transform(std::vector<double> & f)
{
  const int n = static_cast<int>(f.size());
  std::vector<int> v(n);
  std::vector<double> z(n + 1), out(n);
  int k = 0; v[0] = 0; z[0] = -kInfinity; z[1] = kInfinity;
  for (int q = 1; q < n; ++q) {
    double s;
    do {
      const int p = v[k];
      s = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2.0 * (q - p));
      if (s > z[k]) {break;}
      --k;
    } while (k >= 0);
    ++k; v[k] = q; z[k] = s; z[k + 1] = kInfinity;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    out[q] = squared({static_cast<double>(q - v[k]), 0}) + f[v[k]];
  }
  f.swap(out);
}

void inflate(Grid & g, double padding)
{
  std::vector<double> distances(g.occupied.size());
  for (std::size_t i = 0; i < distances.size(); ++i) {distances[i] = g.occupied[i] ? 0 : kInfinity;}
  for (int y = 0; y < g.height; ++y) {
    std::vector<double> row(distances.begin() + y * g.width, distances.begin() + (y + 1) * g.width);
    distance_transform(row);
    std::copy(row.begin(), row.end(), distances.begin() + y * g.width);
  }
  for (int x = 0; x < g.width; ++x) {
    std::vector<double> column(g.height);
    for (int y = 0; y < g.height; ++y) {column[y] = distances[g.index(x, y)];}
    distance_transform(column);
    for (int y = 0; y < g.height; ++y) {distances[g.index(x, y)] = column[y];}
  }
  g.blocked.resize(distances.size()); g.clearance.resize(distances.size());
  for (std::size_t i = 0; i < distances.size(); ++i) {
    const auto p = g.center(static_cast<int>(i));
    // Subtract an occupied cell's half diagonal: conservative distance to its area.
    g.clearance[i] = std::min({std::max(0.0, std::sqrt(distances[i]) * g.res - g.res / std::sqrt(2.0)),
      p.x, p.y, g.width * g.res - p.x, g.height * g.res - p.y});
    g.blocked[i] = g.clearance[i] <= padding + 1e-12;
  }
}

std::string token(std::istream & stream)
{
  std::string result;
  while (stream >> std::ws && stream.peek() == '#') {stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');}
  stream >> result;
  require(!result.empty(), "Truncated PGM image");
  return result;
}

void load_image(Grid & g, const std::filesystem::path & path, const YAML::Node & yaml, const PathPlannerConfig & config)
{
  std::vector<uint8_t> rgba;
  std::vector<double> gray;
  if (path.extension() == ".png") {
    png_image image{}; image.version = PNG_IMAGE_VERSION;
    require(png_image_begin_read_from_file(&image, path.c_str()) != 0, "Cannot read PNG map");
    try {allocate(g, image.width, image.height, config);} catch (...) {png_image_free(&image); throw;}
    image.format = PNG_FORMAT_RGBA; rgba.resize(g.occupied.size() * 4);
    const bool ok = png_image_finish_read(&image, nullptr, rgba.data(), 0, nullptr) != 0;
    png_image_free(&image); require(ok, "Invalid PNG map data");
    gray.resize(g.occupied.size());
    for (std::size_t i = 0; i < gray.size(); ++i) {gray[i] = (rgba[i * 4] + rgba[i * 4 + 1] + rgba[i * 4 + 2]) / (3.0 * 255);}
  } else {
    std::ifstream image(path, std::ios::binary);
    const auto magic = token(image);
    require(magic == "P2" || magic == "P5", "Map image must be PGM or PNG");
    const int width = std::stoi(token(image)), height = std::stoi(token(image));
    const int maximum = std::stoi(token(image));
    require(maximum > 0 && maximum <= 65535, "Invalid PGM maximum value");
    allocate(g, width, height, config); rgba.resize(g.occupied.size() * 4, 255); gray.resize(g.occupied.size());
    if (magic == "P5") {
      const int separator = image.get();
      require(separator != EOF && std::isspace(static_cast<unsigned char>(separator)), "Invalid PGM binary separator");
      if (separator == '\r' && image.peek() == '\n') {image.get();}
    }
    for (std::size_t i = 0; i < g.occupied.size(); ++i) {
      int value;
      if (magic == "P2") {value = std::stoi(token(image));} else {
        const int high = image.get(); require(high != EOF, "Truncated PGM pixels"); value = high;
        if (maximum > 255) {const int low = image.get(); require(low != EOF, "Truncated PGM pixels"); value = high * 256 + low;}
      }
      require(value >= 0 && value <= maximum, "PGM pixel outside its declared range");
      gray[i] = static_cast<double>(value) / maximum;
    }
  }
  const int negate = yaml["negate"] ? yaml["negate"].as<int>() : 0;
  const double free = yaml["free_thresh"] ? yaml["free_thresh"].as<double>() : 0.196;
  const double occupied = yaml["occupied_thresh"] ? yaml["occupied_thresh"].as<double>() : config.occupancy_threshold / 100.0;
  require((negate == 0 || negate == 1) && std::isfinite(free) && std::isfinite(occupied) &&
    free >= 0 && free < occupied && occupied <= 1, "Invalid ROS map thresholds");
  for (int row = 0; row < g.height; ++row) {
    for (int x = 0; x < g.width; ++x) {
      const auto pixel = static_cast<std::size_t>(row * g.width + x) * 4;
      const double intensity = gray[pixel / 4];
      // Unknown gray values and transparency are blocked, as are occupied pixels.
      g.occupied[g.index(x, g.height - 1 - row)] = (negate ? intensity : 1 - intensity) >= free - 1e-12 || rgba[pixel + 3] < 255;
    }
  }
}

void load_text(Grid & g, const PathPlannerConfig & config)
{
  std::ifstream file(config.map_file); require(file.good(), "Cannot open block map");
  std::vector<std::string> rows;
  std::string row;
  std::size_t width = 0;
  while (std::getline(file, row)) {
    const auto first = row.find_first_not_of(" \t\r"), last = row.find_last_not_of(" \t\r");
    if (first == std::string::npos) {continue;}
    row = row.substr(first, last - first + 1);
    if (row.find_first_not_of("#.SGrlud") == std::string::npos) {rows.push_back(row); width = std::max(width, row.size());}
    require(rows.size() <= config.max_cells && width <= config.max_cells, "Block map exceeds max_cells");
  }
  const double ratio = config.block_size / g.res;
  require(std::isfinite(ratio) && ratio >= 1 && std::abs(ratio - std::round(ratio)) < 1e-8, "block_size must be an integer multiple of res");
  allocate(g, width * std::round(ratio), rows.size() * std::round(ratio), config);
  const int k = static_cast<int>(std::round(ratio));
  for (int y = 0; y < g.height; ++y) {
    const auto & source = rows[rows.size() - 1 - y / k];
    for (int x = 0; x < g.width; ++x) {g.occupied[g.index(x, y)] = static_cast<std::size_t>(x / k) >= source.size() || source[x / k] == '#';}
  }
}

uint32_t little_u32(const unsigned char * p)
{
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 |
         static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24;
}

void load_stl(Grid & g, const PathPlannerConfig & config)
{
  std::ifstream file(config.map_file, std::ios::binary | std::ios::ate);
  require(file.good() && file.tellg() >= 84, "Invalid binary STL header");
  const auto size = static_cast<uint64_t>(file.tellg()); file.seekg(0);
  std::array<unsigned char, 84> header{}; file.read(reinterpret_cast<char *>(header.data()), header.size());
  const uint32_t count = little_u32(header.data() + 80);
  require(count > 0 && size == 84ULL + 50ULL * count, "Binary STL triangle count is invalid (ASCII unsupported)");
  std::vector<std::pair<Point, Point>> segments;
  const double infinity = std::numeric_limits<double>::infinity();
  Point minimum{infinity, infinity}, maximum{-infinity, -infinity};
  for (uint32_t triangle = 0; triangle < count; ++triangle) {
    std::array<unsigned char, 50> bytes{}; file.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
    std::array<std::array<double, 3>, 3> vertices{};
    for (int v = 0; v < 3; ++v) {
      for (int c = 0; c < 3; ++c) {
        const uint32_t bits = little_u32(bytes.data() + 12 + (v * 3 + c) * 4);
        float value; std::memcpy(&value, &bits, sizeof(value));
        require(std::isfinite(value), "STL contains non-finite vertices"); vertices[v][c] = value;
      }
    }
    for (const double height : config.stl_slice_heights) {
      const double lo = std::min({vertices[0][2], vertices[1][2], vertices[2][2]});
      const double hi = std::max({vertices[0][2], vertices[1][2], vertices[2][2]});
      if (!(lo < height && height < hi)) {continue;}
      std::vector<Point> hits;
      for (int e = 0; e < 3; ++e) {
        const auto & a = vertices[e]; const auto & b = vertices[(e + 1) % 3];
        if (a[2] == b[2] || height < std::min(a[2], b[2]) || height > std::max(a[2], b[2])) {continue;}
        const double t = (height - a[2]) / (b[2] - a[2]);
        Point p{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
        if (hits.empty() || squared(p - hits.front()) > 1e-20) {hits.push_back(p);}
      }
      if (hits.size() < 2) {continue;}
      segments.emplace_back(hits[0], hits[1]);
      for (const auto p : {hits[0], hits[1]}) {
        minimum.x = std::min(minimum.x, p.x); minimum.y = std::min(minimum.y, p.y);
        maximum.x = std::max(maximum.x, p.x); maximum.y = std::max(maximum.y, p.y);
      }
    }
  }
  require(!segments.empty(), "STL has no wall intersections at configured slice heights");
  g.ox = minimum.x; g.oy = minimum.y;
  allocate(g, std::floor((maximum.x - g.ox) / g.res + 1e-8) + 1,
    std::floor((maximum.y - g.oy) / g.res + 1e-8) + 1, config);
  for (const auto & segment : segments) {
    const auto a = segment.first, b = segment.second;
    const auto steps = static_cast<std::size_t>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / (g.res / 4)));
    for (std::size_t i = 0; i <= steps; ++i) {
      const auto p = lerp(a, b, steps ? static_cast<double>(i) / steps : 0);
      const int x = std::clamp(static_cast<int>(std::floor((p.x - g.ox) / g.res + 1e-9)), 0, g.width - 1);
      const int y = std::clamp(static_cast<int>(std::floor((p.y - g.oy) / g.res + 1e-9)), 0, g.height - 1);
      g.occupied[g.index(x, y)] = 1;
    }
  }
}

double pose_yaw(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position; const auto & q = pose.orientation;
  require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) &&
    std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w), "Pose contains non-finite values");
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  require(std::isfinite(norm) && norm >= 1e-9, "Pose quaternion is zero or invalid");
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

Grid obstacle_snapshot(const Grid & original, const geometry_msgs::msg::PoseArray & obstacles, const PathPlannerConfig & config)
{
  require(obstacles.poses.empty() || obstacles.header.frame_id == config.map_frame, "Obstacle frame must match map_frame");
  Grid grid = original;
  for (const auto & p : obstacles.poses) {
    require(std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z), "Obstacle position is not finite");
    const int cell = grid.cell(grid.local(p.position));
    if (cell >= 0) {grid.occupied[cell] = 1;}
  }
  if (!obstacles.poses.empty()) {inflate(grid, config.r_robot + config.margin);}
  return grid;
}

double point_segment_distance_squared(Point p, Point a, Point b)
{
  const auto d = b - a; const double length = squared(d);
  const double t = length > 0 ? std::clamp(((p.x - a.x) * d.x + (p.y - a.y) * d.y) / length, 0.0, 1.0) : 0;
  return squared(p - lerp(a, b, t));
}

double segment_rectangle_distance_squared(Point a, Point b, double x, double y, double size)
{
  double enter = 0, leave = 1;
  const auto slab = [&enter, &leave](double start, double delta, double lo, double hi) {
      if (std::abs(delta) < 1e-15) {return start >= lo && start <= hi;}
      double u = (lo - start) / delta, v = (hi - start) / delta;
      if (u > v) {std::swap(u, v);} enter = std::max(enter, u); leave = std::min(leave, v);
      return enter <= leave;
    };
  if (slab(a.x, b.x - a.x, x, x + size) && slab(a.y, b.y - a.y, y, y + size)) {return 0;}
  double result = kInfinity;
  for (auto p : {a, b}) {result = std::min(result, squared({p.x - std::clamp(p.x, x, x + size), p.y - std::clamp(p.y, y, y + size)}));}
  for (const Point p : {Point{x, y}, Point{x + size, y}, Point{x, y + size}, Point{x + size, y + size}}) {
    result = std::min(result, point_segment_distance_squared(p, a, b));
  }
  return result;
}

bool segment_free(const Grid & g, Point a, Point b, double padding)
{
  for (const auto p : {a, b}) {
    if (!(p.x > padding && p.y > padding && p.x < g.width * g.res - padding && p.y < g.height * g.res - padding)) {return false;}
  }
  // Subdivide for a narrow candidate-cell box, but use exact segment/rectangle distance
  // within each box. Tiny collisions between samples cannot slip through.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / g.res)));
  for (int step = 0; step < steps; ++step) {
    const auto p = lerp(a, b, static_cast<double>(step) / steps), q = lerp(a, b, static_cast<double>(step + 1) / steps);
    const int x0 = std::max(0, static_cast<int>(std::floor((std::min(p.x, q.x) - padding) / g.res - 1e-9)));
    const int x1 = std::min(g.width - 1, static_cast<int>(std::floor((std::max(p.x, q.x) + padding) / g.res + 1e-9)));
    const int y0 = std::max(0, static_cast<int>(std::floor((std::min(p.y, q.y) - padding) / g.res - 1e-9)));
    const int y1 = std::min(g.height - 1, static_cast<int>(std::floor((std::max(p.y, q.y) + padding) / g.res + 1e-9)));
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (g.occupied[g.index(x, y)] && segment_rectangle_distance_squared(p, q, x * g.res, y * g.res, g.res) <= padding * padding + 1e-14) {return false;}
      }
    }
  }
  return true;
}

std::vector<Point> astar(const Grid & g, Point start, Point goal, double start_yaw, const PathPlannerConfig & config)
{
  const int first = g.cell(start), last = g.cell(goal);
  const double padding = config.r_robot + config.margin;
  require(first >= 0 && last >= 0, "Start or goal is outside the map");
  if (g.blocked[first] || g.blocked[last] || !segment_free(g, start, start, padding) ||
    !segment_free(g, goal, goal, padding))
  {
    throw RouteUnavailable("Start or goal is in an inflated obstacle");
  }
  constexpr std::array<int, 8> dx{1, 1, 0, -1, -1, -1, 0, 1}, dy{0, 1, 1, 1, 0, -1, -1, -1};
  struct Entry {double f, cost; int state; bool operator<(const Entry & other) const {return f > other.f;}};
  std::priority_queue<Entry> open;
  std::vector<double> costs(g.occupied.size() * 8, std::numeric_limits<double>::infinity());
  std::vector<int> parent(costs.size(), -1);
  const int heading = (static_cast<int>(std::lround(std::remainder(start_yaw, 2 * kPi) / (kPi / 4))) % 8 + 8) % 8;
  const int initial = first * 8 + heading;
  const auto heuristic = [&g, last](int cell) {const auto d = g.center(cell) - g.center(last); return std::hypot(d.x, d.y);};
  costs[initial] = 0; open.push({heuristic(first), 0, initial});
  int final = -1; std::size_t expansions = 0;
  while (!open.empty()) {
    const auto current = open.top(); open.pop();
    if (current.cost != costs[current.state]) {continue;}
    require(++expansions <= config.max_expansions, "A* exceeded max_expansions");
    const int cell = current.state / 8, x = cell % g.width, y = cell / g.width, h = current.state % 8;
    if (cell == last) {final = current.state; break;}
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + dx[direction], ny = y + dy[direction];
      if (!g.contains(nx, ny) || g.blocked[g.index(nx, ny)]) {continue;}
      if (dx[direction] && dy[direction] && (g.blocked[g.index(nx, y)] || g.blocked[g.index(x, ny)])) {continue;}
      const int next_cell = g.index(nx, ny), next = next_cell * 8 + direction;
      const int difference = std::abs(direction - h), turn = std::min(difference, 8 - difference);
      const double proximity = std::clamp((padding + config.prox_band - g.clearance[next_cell]) / config.prox_band, 0.0, 1.0);
      const double candidate = current.cost + g.res * (direction % 2 ? std::sqrt(2.0) : 1.0) *
        (1 + config.w_prox * proximity) + config.lam * turn / 2.0;
      if (candidate < costs[next]) {costs[next] = candidate; parent[next] = current.state; open.push({candidate + heuristic(next_cell), candidate, next});}
    }
  }
  if (final < 0) {throw RouteUnavailable("No route through the inflated map");}
  std::vector<Point> reversed;
  for (int state = final; state >= 0; state = parent[state]) {reversed.push_back(g.center(state / 8));}
  std::reverse(reversed.begin(), reversed.end());
  reversed.insert(reversed.begin(), start); reversed.push_back(goal);
  // Farthest visible shortcut only; no unverified spline or corner-cutting fillet.
  std::vector<Point> result{start};
  for (std::size_t anchor = 0; anchor + 1 < reversed.size();) {
    std::size_t next = reversed.size() - 1;
    while (next > anchor + 1 && !segment_free(g, reversed[anchor], reversed[next], padding)) {--next;}
    require(segment_free(g, reversed[anchor], reversed[next], padding), "No collision-free connection between path samples");
    if (squared(reversed[next] - result.back()) > 1e-20) {result.push_back(reversed[next]);}
    anchor = next;
  }
  return result;
}
}  // namespace

struct PathPlanner::Data {PathPlannerConfig config; Grid grid;};

void PathPlanner::configure(const PathPlannerConfig & config)
{
  try {
    require(!config.map_file.empty() && !config.map_frame.empty(), "map_file and map_frame are required");
    for (const double p : {config.res, config.block_size, config.prox_band, config.arc_sample}) {require(std::isfinite(p) && p > 0, "Planner lengths must be positive and finite");}
    for (const double p : {config.r_robot, config.margin, config.lam, config.w_prox}) {require(std::isfinite(p) && p >= 0, "Planner costs and footprint must be nonnegative and finite");}
    require(std::isfinite(config.r_robot + config.margin) && config.max_cells > 0 &&
      config.max_cells <= static_cast<std::size_t>(std::numeric_limits<int>::max() / 8) && config.max_expansions > 0 &&
      config.occupancy_threshold >= 1 && config.occupancy_threshold <= 100, "Invalid planner resource limit or occupancy threshold");
    require(!config.stl_slice_heights.empty(), "STL slice heights must be nonempty");
    for (const double z : config.stl_slice_heights) {require(std::isfinite(z), "STL slice height is not finite");}
    auto data = std::make_shared<Data>(); data->config = config; auto & grid = data->grid; grid.res = config.res;
    const auto path = std::filesystem::path(config.map_file);
    if (path.extension() == ".stl") {load_stl(grid, config);} else if (path.extension() == ".txt") {load_text(grid, config);} else {
      require(path.extension() == ".yaml" || path.extension() == ".yml", "Use binary STL, text block map or ROS YAML map");
      const auto yaml = YAML::LoadFile(config.map_file);
      require(!yaml["mode"] || yaml["mode"].as<std::string>() == "trinary", "Only trinary ROS maps are supported");
      const auto origin = yaml["origin"].as<std::vector<double>>();
      require(origin.size() == 3, "Map origin must contain x, y and yaw");
      for (const auto p : origin) {require(std::isfinite(p), "Map origin is not finite");}
      grid.ox = origin[0]; grid.oy = origin[1]; grid.yaw = origin[2]; grid.res = yaml["resolution"].as<double>();
      require(std::isfinite(grid.res) && grid.res > 0, "Map resolution must be positive and finite");
      load_image(grid, path.parent_path() / yaml["image"].as<std::string>(), yaml, config);
    }
    require(std::isfinite(grid.width * grid.res) && std::isfinite(grid.height * grid.res), "Map extent is not finite");
    inflate(grid, config.r_robot + config.margin);
    std::atomic_store(&data_, std::shared_ptr<const Data>(data));
  } catch (const std::exception & error) {throw std::invalid_argument(error.what());}
}

PathPlan PathPlanner::plan(
  const nav_msgs::msg::Odometry & odom, const geometry_msgs::msg::PoseStamped & goal,
  const geometry_msgs::msg::PoseArray & map_obstacles) const
{
  PathPlan result;
  try {
    const auto data = std::atomic_load(&data_); require(data != nullptr, "Planner has not been configured");
    const auto & config = data->config;
    require(odom.header.frame_id == config.map_frame && goal.header.frame_id == config.map_frame, "Map, goal and odometry must use the same frame");
    const double start_yaw = pose_yaw(odom.pose.pose), goal_yaw = pose_yaw(goal.pose);
    const auto grid = obstacle_snapshot(data->grid, map_obstacles, config);
    const auto points = astar(grid, grid.local(odom.pose.pose.position), grid.local(goal.pose.position), start_yaw - grid.yaw, config);
    result.path.header.frame_id = config.map_frame;
    const double spacing = std::min(config.arc_sample, grid.res / 2);
    const auto append = [&result, &grid, &config](Point p, double yaw) {
        geometry_msgs::msg::PoseStamped pose; pose.header.frame_id = config.map_frame;
        pose.pose.position.x = grid.ox + std::cos(grid.yaw) * p.x - std::sin(grid.yaw) * p.y;
        pose.pose.position.y = grid.oy + std::sin(grid.yaw) * p.x + std::cos(grid.yaw) * p.y;
        pose.pose.orientation.z = std::sin(yaw / 2); pose.pose.orientation.w = std::cos(yaw / 2);
        result.path.poses.push_back(pose);
      };
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto a = points[i - 1], b = points[i]; const auto d = b - a;
      const double count = std::ceil(std::hypot(d.x, d.y) / spacing);
      const auto limit = std::min(config.max_cells * 16, config.max_expansions);
      require(std::isfinite(count) && count <= limit && result.path.poses.size() <= limit - count,
        "Path sampling exceeds the map/work resource limit");
      const auto steps = static_cast<std::size_t>(count);
      for (std::size_t s = 0; s < steps; ++s) {append(lerp(a, b, static_cast<double>(s) / steps), std::atan2(d.y, d.x) + grid.yaw);}
    }
    append(points.back(), goal_yaw);
    // Preserve requested XY exactly, including floating-point map-origin transforms.
    result.path.poses.front().pose.position.x = odom.pose.pose.position.x;
    result.path.poses.front().pose.position.y = odom.pose.pose.position.y;
    result.path.poses.back().pose.position.x = goal.pose.position.x;
    result.path.poses.back().pose.position.y = goal.pose.position.y;
    result.ok = true;
  } catch (const RouteUnavailable & error) {
    result.path = nav_msgs::msg::Path(); result.error = error.what(); result.retryable = true;
  } catch (const std::exception & error) {
    result.path = nav_msgs::msg::Path(); result.error = error.what();
  }
  return result;
}

bool PathPlanner::collision_free(const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseArray & map_obstacles) const
{
  try {
    const auto data = std::atomic_load(&data_);
    if (!data || path.poses.empty() || path.header.frame_id != data->config.map_frame) {return false;}
    const auto grid = obstacle_snapshot(data->grid, map_obstacles, data->config);
    Point previous{};
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i]; pose_yaw(pose.pose);
      if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {return false;}
      const auto current = grid.local(pose.pose.position);
      if (!segment_free(grid, i ? previous : current, current, data->config.r_robot + data->config.margin)) {return false;}
      previous = current;
    }
    return true;
  } catch (const std::exception &) {return false;}
}
}  // namespace craft_planning
