#include "craft_planning/path_planner.hpp"

#include <png.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
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
    // A*+arc ESDF: centre-to-centre distance minus half a cell; outside the map counts as occupied.
    g.clearance[i] = std::min({std::max(std::sqrt(distances[i]) * g.res - 0.5 * g.res, 0.0),
      p.x, p.y, g.width * g.res - p.x, g.height * g.res - p.y});
    g.blocked[i] = g.occupied[i] || g.clearance[i] < padding;  // occupied stays blocked with zero padding
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

// ---------------------------------------------------------------------------------------------------------
// A*+arc, ported from astar_arc.py (same costs, heap tie order and fillet search).
// Grid cells are A*+arc cells: local metres = (index + 0.5) * res; point checks use the inflated grid.
// ---------------------------------------------------------------------------------------------------------
class NoRoute : public RouteUnavailable
{
public:
  using RouteUnavailable::RouteUnavailable;
};
constexpr std::array<int, 4> kDx{1, 0, -1, 0}, kDy{0, 1, 0, -1};  // heading 0=E, 1=N, 2=W, 3=S
constexpr std::array<double, 4> kHeading{0.0, kPi / 2, kPi, -kPi / 2};
const char * const kStopTurn = "STOP_TURN", * const kPass = "PASS", * const kAlign = "ALIGN";
int wrap4(int h) {return (h % 4 + 4) % 4;}
Point operator+(Point a, Point b) {return {a.x + b.x, a.y + b.y};}
Point operator*(Point a, double k) {return {a.x * k, a.y * k};}

struct State {int x, y, h;};
struct Corner {int x, y, h_in, h_out;};
struct Waypoint
{
  Point p; double theta; std::string mode; double radius = 0, v_max = 0; std::string reason;
  int h_in = -1, h_out = -1;
};
struct Sample {Point p; double yaw, kappa, v;};
struct Leg
{
  std::vector<State> states; std::vector<Waypoint> waypoints; std::vector<Sample> path;
  std::vector<std::size_t> stops; std::size_t expanded = 0;
};

bool point_free(const Grid & g, Point p)
{
  const int x = static_cast<int>(std::floor(p.x / g.res)), y = static_cast<int>(std::floor(p.y / g.res));
  return g.contains(x, y) && !g.blocked[g.index(x, y)];
}

int min_turns(int h, int dx, int dy)
{
  std::array<bool, 4> need{dx > 0, dy > 0, dx < 0, dy < 0};
  const int count = need[0] + need[1] + need[2] + need[3];
  if (!count) {return 0;}
  if (need[h]) {return count - 1;}
  if (count == 1) {for (int d = 0; d < 4; ++d) {if (need[d]) {return wrap4(d - h) == 2 ? 2 : 1;}}}
  return 2;
}

// (start, start_h) -> goal; with goal_h >= 0 the goal counts only at that heading. Forward one cell costs
// res * (1 + proximity), a 90 degree turn in place costs lam.
std::vector<State> astar_arc_search(
  const Grid & g, const PathPlannerConfig & c, State start, int gx, int gy, int goal_h, std::size_t & expanded)
{
  if (g.blocked[g.index(start.x, start.y)]) {throw RouteUnavailable("Start is in an inflated obstacle");}
  if (g.blocked[g.index(gx, gy)]) {throw RouteUnavailable("Goal is in an inflated obstacle");}
  const double pad = c.r_robot + c.margin;
  const auto hfun = [&](int x, int y, int h) {
      const int dx = gx - x, dy = gy - y;
      int t = min_turns(h, dx, dy);
      if (goal_h >= 0 && dx == 0 && dy == 0 && h != goal_h) {t = wrap4(goal_h - h) == 2 ? 2 : 1;}
      return g.res * (std::abs(dx) + std::abs(dy)) + c.lam * t;
    };
  struct Entry
  {
    double f, cost; int x, y, h;
    bool operator>(const Entry & o) const {return std::tie(f, cost, x, y, h) > std::tie(o.f, o.cost, o.x, o.y, o.h);}
  };
  const auto id = [&g](int x, int y, int h) {return static_cast<std::size_t>(g.index(x, y)) * 4 + h;};
  std::vector<double> best(g.occupied.size() * 4, std::numeric_limits<double>::infinity());
  std::vector<int64_t> parent(best.size(), -1);
  std::vector<uint8_t> closed(best.size(), 0);
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  best[id(start.x, start.y, start.h)] = 0;
  open.push({hfun(start.x, start.y, start.h), 0.0, start.x, start.y, start.h});
  int64_t goal_state = -1;
  while (!open.empty()) {
    const auto e = open.top(); open.pop();
    const auto here = id(e.x, e.y, e.h);
    if (closed[here]) {continue;}
    closed[here] = 1;
    require(++expanded <= c.max_expansions, "A* exceeded max_expansions");
    if (e.x == gx && e.y == gy && (goal_h < 0 || e.h == goal_h)) {goal_state = static_cast<int64_t>(here); break;}
    std::array<Entry, 3> next{};
    int count = 0;
    const int nx = e.x + kDx[e.h], ny = e.y + kDy[e.h];
    if (g.contains(nx, ny) && !g.blocked[g.index(nx, ny)]) {
      const double prox = std::clamp((pad + c.prox_band - g.clearance[g.index(nx, ny)]) / c.prox_band, 0.0, 1.0) * c.w_prox;
      next[count++] = {0, g.res * (1.0 + prox), nx, ny, e.h};
    }
    next[count++] = {0, c.lam, e.x, e.y, wrap4(e.h + 1)};
    next[count++] = {0, c.lam, e.x, e.y, wrap4(e.h - 1)};
    for (int k = 0; k < count; ++k) {
      const auto & n = next[k];
      const double cost = e.cost + n.cost;
      const auto there = id(n.x, n.y, n.h);
      if (cost < best[there]) {
        best[there] = cost; parent[there] = static_cast<int64_t>(here);
        open.push({cost + hfun(n.x, n.y, n.h), cost, n.x, n.y, n.h});
      }
    }
  }
  if (goal_state < 0) {throw NoRoute("No route through the inflated map");}
  std::vector<State> states;
  for (int64_t s = goal_state; s >= 0; s = parent[s]) {
    const int cell = static_cast<int>(s / 4);
    states.push_back({cell % g.width, cell / g.width, static_cast<int>(s % 4)});
  }
  std::reverse(states.begin(), states.end());
  return states;
}

// A* states -> [start, corner..., goal] with the heading before and after each.
std::vector<Corner> compress_corners(const std::vector<State> & states)
{
  std::vector<State> moves;  // forward steps: leave (x, y) along h
  for (std::size_t i = 0; i + 1 < states.size(); ++i) {
    if (states[i + 1].x != states[i].x || states[i + 1].y != states[i].y) {moves.push_back(states[i]);}
  }
  const auto & s0 = states.front();
  std::vector<Corner> pts{{s0.x, s0.y, s0.h, moves.empty() ? s0.h : moves.front().h}};
  for (std::size_t i = 0; i + 1 < moves.size(); ++i) {
    if (moves[i + 1].h != moves[i].h) {pts.push_back({moves[i + 1].x, moves[i + 1].y, moves[i].h, moves[i + 1].h});}
  }
  const auto & last = states.back();
  pts.push_back({last.x, last.y, moves.empty() ? s0.h : moves.back().h, last.h});
  return pts;
}

struct Fillet {std::vector<Point> points; Point t1, t2; double a0, sweep, turn;};
Fillet make_fillet(Point corner, Point u_in, Point u_out, double r, double ds)
{
  Fillet f;
  f.t1 = corner - u_in * r; f.t2 = corner + u_out * r;
  const Point o = corner - u_in * r + u_out * r;
  const double cross = u_in.x * u_out.y - u_in.y * u_out.x;
  f.turn = (cross > 0) - (cross < 0);
  f.a0 = std::atan2(f.t1.y - o.y, f.t1.x - o.x); f.sweep = f.turn * kPi / 2;
  const double count = std::ceil(std::abs(f.sweep) * r / ds) + 1;
  require(std::isfinite(count) && count <= 1e6, "Path sampling exceeds the map/work resource limit");
  const int n = std::max(2, static_cast<int>(count));
  for (int i = 0; i < n; ++i) {
    const double a = f.a0 + (i == n - 1 ? f.sweep : f.sweep * i / (n - 1));
    f.points.push_back({o.x + r * std::cos(a), o.y + r * std::sin(a)});
  }
  return f;
}

bool arc_free(const Grid & g, const std::vector<Point> & points)
{
  return std::all_of(points.begin(), points.end(), [&g](Point p) {return point_free(g, p);});
}

double max_fillet_radius(const Grid & g, const PathPlannerConfig & c, Point corner, Point u_in, Point u_out,
  double in_avail, double out_avail, double arc_sample)
{
  double hi = std::min({c.r_hi_global, in_avail, out_avail}), lo = 0.0;
  if (hi <= 0) {return 0.0;}
  if (arc_free(g, make_fillet(corner, u_in, u_out, hi, arc_sample).points)) {return hi;}
  while (hi - lo > c.r_tol) {
    const double r = 0.5 * (lo + hi);
    (arc_free(g, make_fillet(corner, u_in, u_out, r, arc_sample).points) ? lo : hi) = r;
  }
  return lo;
}

double v_for_radius(const PathPlannerConfig & c, double r)
{
  return r <= 0 ? 0.0 : std::min({c.v_ref, c.w_max * r, std::sqrt(c.a_lat_max * r)});
}

std::string fixed3(double v) {char text[32]; std::snprintf(text, sizeof(text), "%.3f", v); return text;}

// approach > 0 keeps the last `approach` metres straight (no fillet) and ends in ALIGN.
std::vector<Waypoint> build_waypoints(const Grid & g, const PathPlannerConfig & c, const std::vector<State> & states,
  int goal_heading, double approach, double arc_sample)
{
  const auto pts = compress_corners(states);
  const std::size_t n = pts.size();
  std::vector<Point> xy;
  for (const auto & p : pts) {xy.push_back(g.center(g.index(p.x, p.y)));}
  static const std::array<const char *, 4> dir_name{"r", "u", "l", "d"};
  std::vector<std::string> forced(n);
  forced[0] = "start";
  forced[n - 1] = approach > 0 ? std::string("goal ALIGN (") + dir_name[goal_heading] + ")" : "goal";
  for (std::size_t i = 1; i + 1 < n; ++i) {if (wrap4(pts[i].h_out - pts[i].h_in) == 2) {forced[i] = "180° reversal";}}
  std::vector<double> r_used(n, 0.0);
  std::vector<Waypoint> wps;
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = pts[i];
    if (!forced[i].empty()) {
      const int th = i == n - 1 && goal_heading >= 0 ? goal_heading : p.h_out;
      wps.push_back({xy[i], kHeading[th], i == n - 1 && approach > 0 ? kAlign : kStopTurn, 0, 0, forced[i], p.h_in, th});
      continue;
    }
    const double len_in = std::abs(xy[i].x - xy[i - 1].x) + std::abs(xy[i].y - xy[i - 1].y);
    const double len_out = std::abs(xy[i + 1].x - xy[i].x) + std::abs(xy[i + 1].y - xy[i].y);
    const double in_avail = len_in - r_used[i - 1];
    double out_avail = forced[i + 1].empty() ? 0.5 * len_out : len_out;
    if (i == n - 2 && approach > 0) {out_avail = len_out - approach;}
    if (out_avail <= 1e-9) {
      wps.push_back({xy[i], kHeading[p.h_out], kStopTurn, 0, 0, "approach entry (turn in place)", p.h_in, p.h_out});
      continue;
    }
    const Point u_in{static_cast<double>(kDx[p.h_in]), static_cast<double>(kDy[p.h_in])};
    const Point u_out{static_cast<double>(kDx[p.h_out]), static_cast<double>(kDy[p.h_out])};
    const double r = max_fillet_radius(g, c, xy[i], u_in, u_out, in_avail, out_avail, arc_sample);
    std::string limit = r >= c.r_hi_global - 1e-9 ? "global" : r >= std::min(in_avail, out_avail) - 1e-9 ? "segment" : "obstacle";
    if (r >= c.r_min) {
      r_used[i] = r;
      if (i == n - 2 && approach > 0 && r >= out_avail - 1e-9) {limit = "approach";}
      wps.push_back({xy[i], kHeading[p.h_out], kPass, r, v_for_radius(c, r), "R limited by " + limit, p.h_in, p.h_out});
    } else {
      wps.push_back({xy[i], kHeading[p.h_out], kStopTurn, 0, 0, "R=" + fixed3(r) + " < R_MIN (" + limit + ")", p.h_in, p.h_out});
    }
  }
  return wps;
}

// Lines and fillets sampled every ds; stops = sample indices of intermediate STOP_TURN corners.
void build_path(const PathPlannerConfig & c, Leg & leg, double ds = 0.02)
{
  auto & out = leg.path;
  const auto & wps = leg.waypoints;
  Point cur = wps.front().p;
  const auto add_line = [&out, ds](Point a, Point b, double yaw, double v) {
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      if (length < 1e-9) {return;}
      const int k = std::max(1, static_cast<int>(std::ceil(length / ds)));
      for (int i = 0; i < k; ++i) {out.push_back({lerp(a, b, static_cast<double>(i) / k), yaw, 0.0, v});}
    };
  for (std::size_t i = 1; i < wps.size(); ++i) {
    const auto & w = wps[i];
    const double yaw_in = w.h_in >= 0 ? kHeading[w.h_in] : wps[i - 1].theta;
    if (w.mode == kPass) {
      const Point u_in{static_cast<double>(kDx[w.h_in]), static_cast<double>(kDy[w.h_in])};
      const Point u_out{static_cast<double>(kDx[w.h_out]), static_cast<double>(kDy[w.h_out])};
      const auto f = make_fillet(w.p, u_in, u_out, w.radius, ds);
      add_line(cur, f.t1, yaw_in, c.v_ref);
      for (std::size_t j = 0; j + 1 < f.points.size(); ++j) {
        const double yaw = f.a0 + f.sweep * j / (f.points.size() - 1) + f.turn * kPi / 2;
        out.push_back({f.points[j], std::atan2(std::sin(yaw), std::cos(yaw)), f.turn / w.radius, w.v_max});
      }
      cur = f.t2;
    } else {
      add_line(cur, w.p, yaw_in, c.v_ref);
      cur = w.p;
      if (i + 1 < wps.size()) {leg.stops.push_back(out.size());}
    }
  }
  out.push_back({wps.back().p, wps.back().theta, 0.0, 0.0});
  // A zero-length line can leave a stop index past the next sample; keep only real interior samples.
  leg.stops.erase(std::remove_if(leg.stops.begin(), leg.stops.end(),
    [&out](std::size_t i) {return i == 0 || i + 1 >= out.size();}), leg.stops.end());
  leg.stops.erase(std::unique(leg.stops.begin(), leg.stops.end()), leg.stops.end());
}

double leg_time(const PathPlannerConfig & c, const Leg & leg)
{
  double drive = 0;
  for (std::size_t i = 0; i + 1 < leg.path.size(); ++i) {
    const auto & a = leg.path[i], & b = leg.path[i + 1];
    drive += std::hypot(b.p.x - a.p.x, b.p.y - a.p.y) / std::max(a.v, 1e-3);
  }
  int quarter_turns = 0, stops = 0;
  for (const auto & w : leg.waypoints) {
    if (w.mode != kStopTurn) {continue;}
    if (w.h_in >= 0 && w.h_out >= 0) {quarter_turns += std::min(wrap4(w.h_out - w.h_in), wrap4(w.h_in - w.h_out));}
    stops += w.reason != "start";
  }
  return drive + quarter_turns * (kPi / 2) / c.w_max + stops * c.v_ref / c.a_lon_max;
}

// One goal. With goal_h >= 0: A* to the entry point E = goal - L * u(goal_h) arriving along goal_h, then
// straight E -> goal; L = approach_len + {r_hi, r_hi / 2, 0}, the fastest estimate wins.
Leg plan_leg(const Grid & g, const PathPlannerConfig & c, State start, int gx, int gy, int goal_h, double arc_sample)
{
  Leg leg;
  if (goal_h < 0) {
    leg.states = astar_arc_search(g, c, start, gx, gy, -1, leg.expanded);
    leg.waypoints = build_waypoints(g, c, leg.states, -1, 0.0, arc_sample);
    build_path(c, leg);
    return leg;
  }
  const int n_app = std::max(1, static_cast<int>(std::lround(c.approach_len / g.res)));
  const auto cells = [&](int n) {
      std::vector<std::pair<int, int>> cs;
      for (int i = 0; i <= n; ++i) {cs.push_back({gx - (n - i) * kDx[goal_h], gy - (n - i) * kDy[goal_h]});}
      return cs;
    };
  const auto clear = [&g](const std::vector<std::pair<int, int>> & cs) {
      return std::all_of(cs.begin(), cs.end(), [&g](const auto & p) {
          return g.contains(p.first, p.second) && !g.blocked[g.index(p.first, p.second)];});
    };
  if (!clear({{gx, gy}})) {throw RouteUnavailable("Goal is in an inflated obstacle");}
  if (!clear(cells(n_app))) {throw RouteUnavailable("Goal approach line crosses an inflated obstacle");}
  std::array<int, 3> extras{static_cast<int>(std::lround(c.r_hi_global / g.res)),
    static_cast<int>(std::lround(0.5 * c.r_hi_global / g.res)), 0};
  std::sort(extras.begin(), extras.end(), std::greater<int>());
  std::optional<std::pair<double, Leg>> best;
  std::size_t expanded = 0;
  for (std::size_t k = 0; k < extras.size(); ++k) {
    if (k && extras[k] == extras[k - 1]) {continue;}
    const auto cs = cells(n_app + extras[k]);
    if (!clear(cs)) {continue;}
    Leg candidate;
    try {candidate.states = astar_arc_search(g, c, start, cs[0].first, cs[0].second, goal_h, expanded);} catch (const NoRoute &) {continue;}
    for (std::size_t i = 1; i < cs.size(); ++i) {candidate.states.push_back({cs[i].first, cs[i].second, goal_h});}
    candidate.waypoints = build_waypoints(g, c, candidate.states, goal_h, n_app * g.res, arc_sample);
    build_path(c, candidate);
    const double t = leg_time(c, candidate);
    if (!best || t < best->first - 1e-6) {best.emplace(t, std::move(candidate));}
  }
  if (!best) {throw NoRoute("No route to the goal approach line");}
  leg = std::move(best->second); leg.expanded = expanded;
  return leg;
}
}  // namespace

struct PathPlanner::Data {PathPlannerConfig config; Grid grid;};

void PathPlanner::configure(const PathPlannerConfig & config)
{
  try {
    require(!config.map_file.empty() && !config.map_frame.empty(), "map_file and map_frame are required");
    for (const double p : {config.res, config.block_size, config.prox_band, config.arc_sample}) {require(std::isfinite(p) && p > 0, "Planner lengths must be positive and finite");}
    for (const double p : {config.r_robot, config.margin, config.lam, config.w_prox}) {require(std::isfinite(p) && p >= 0, "Planner costs and footprint must be nonnegative and finite");}
    for (const double p : {config.r_hi_global, config.r_tol, config.v_ref, config.w_max, config.a_lat_max,
         config.a_lon_max, config.approach_len}) {require(std::isfinite(p) && p > 0, "Planner fillet/speed parameters must be positive and finite");}
    require(std::isfinite(config.r_min) && config.r_min >= 0 && config.r_min <= config.r_hi_global, "r_min must be in [0, r_hi_global]");
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
  // A start just inside the margin (tracking error, a robot placed near a wall) leaves straight to the nearest
  // free cell within this distance while the body itself (r_robot) stays clear; it is a STOP_TURN segment.
  constexpr double kEscape = 0.10;
  PathPlan result;
  try {
    const auto data = std::atomic_load(&data_); require(data != nullptr, "Planner has not been configured");
    const auto & config = data->config;
    require(odom.header.frame_id == config.map_frame && goal.header.frame_id == config.map_frame, "Map, goal and odometry must use the same frame");
    const double start_yaw = pose_yaw(odom.pose.pose), goal_yaw = pose_yaw(goal.pose);
    const auto grid = obstacle_snapshot(data->grid, map_obstacles, config);
    const Point start = grid.local(odom.pose.pose.position), target = grid.local(goal.pose.position);
    const int first = grid.cell(start), last = grid.cell(target);
    require(first >= 0 && last >= 0, "Start or goal is outside the map");
    const auto quarter = [&grid](double yaw) {return wrap4(static_cast<int>(std::lround((yaw - grid.yaw) / (kPi / 2))));};
    State from{first % grid.width, first / grid.width, quarter(start_yaw)};
    std::vector<Sample> escape;
    if (grid.blocked[first]) {
      if (grid.clearance[first] < config.r_robot) {throw RouteUnavailable("Start is in an obstacle");}
      const int reach = static_cast<int>(std::ceil(kEscape / grid.res));
      int chosen = -1; double nearest = kInfinity;
      for (int y = from.y - reach; y <= from.y + reach; ++y) {
        for (int x = from.x - reach; x <= from.x + reach; ++x) {
          if (!grid.contains(x, y) || grid.blocked[grid.index(x, y)]) {continue;}
          const Point p = grid.center(grid.index(x, y));
          const double d = std::hypot(p.x - start.x, p.y - start.y);
          if (d > kEscape || d >= nearest) {continue;}
          bool body_clear = true;
          for (int k = 0; k <= std::ceil(d / (grid.res / 2)) && body_clear; ++k) {
            const int cell = grid.cell(lerp(start, p, d > 0 ? std::min(1.0, k * grid.res / 2 / d) : 1.0));
            body_clear = cell >= 0 && grid.clearance[cell] >= config.r_robot;
          }
          if (body_clear) {nearest = d; chosen = grid.index(x, y);}
        }
      }
      if (chosen < 0) {throw RouteUnavailable("Start is in an inflated obstacle with no free cell nearby");}
      const Point p = grid.center(chosen);
      const double yaw = std::atan2(p.y - start.y, p.x - start.x);
      const int k = std::max(1, static_cast<int>(std::ceil(nearest / 0.02)));
      for (int i = 0; i < k; ++i) {escape.push_back({lerp(start, p, static_cast<double>(i) / k), yaw, 0.0, config.v_ref});}
      from.x = chosen % grid.width; from.y = chosen / grid.width;
    }
    const Leg leg = plan_leg(grid, config, from, last % grid.width, last / grid.width,
      config.goal_heading_required ? quarter(goal_yaw) : -1, std::min(config.arc_sample, grid.res / 2));
    std::vector<Sample> samples = escape;
    samples.insert(samples.end(), leg.path.begin(), leg.path.end());
    if (!escape.empty()) {result.stops.push_back(escape.size());}
    for (const auto i : leg.stops) {result.stops.push_back(escape.size() + i);}
    const double c = std::cos(grid.yaw), s = std::sin(grid.yaw);
    const auto to_map = [&grid, c, s](Point p) {return Point{grid.ox + c * p.x - s * p.y, grid.oy + s * p.x + c * p.y};};
    result.path.header.frame_id = config.map_frame;
    for (const auto & sample : samples) {
      geometry_msgs::msg::PoseStamped pose; pose.header.frame_id = config.map_frame;
      const auto p = to_map(sample.p);
      pose.pose.position.x = p.x; pose.pose.position.y = p.y;
      const double yaw = sample.yaw + grid.yaw;
      pose.pose.orientation.z = std::sin(yaw / 2); pose.pose.orientation.w = std::cos(yaw / 2);
      result.path.poses.push_back(pose);
      result.speed_limit.push_back(sample.v);
    }
    if (!escape.empty()) {
      const auto p = to_map(start);
      result.waypoints.push_back({p.x, p.y, escape.front().yaw + grid.yaw, 0, 0, kStopTurn, "escape to the nearest free cell"});
    }
    for (const auto & w : leg.waypoints) {
      const auto p = to_map(w.p);
      result.waypoints.push_back({p.x, p.y, w.theta + grid.yaw, w.radius, w.v_max, w.mode, w.reason});
    }
    // Preserve requested XY exactly, including floating-point map-origin transforms.
    result.path.poses.front().pose.position.x = odom.pose.pose.position.x;
    result.path.poses.front().pose.position.y = odom.pose.pose.position.y;
    result.path.poses.back().pose.position.x = goal.pose.position.x;
    result.path.poses.back().pose.position.y = goal.pose.position.y;
    result.expanded = leg.expanded;
    result.ok = true;
  } catch (const RouteUnavailable & error) {
    result = PathPlan(); result.error = error.what(); result.retryable = true;
  } catch (const std::exception & error) {
    result = PathPlan(); result.error = error.what();
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
      const Point from = i ? previous : current;
      const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(current.x - from.x, current.y - from.y) / (grid.res / 2))));
      for (int k = 0; k <= steps; ++k) {if (!point_free(grid, lerp(from, current, static_cast<double>(k) / steps))) {return false;}}
      previous = current;
    }
    return true;
  } catch (const std::exception &) {return false;}
}
}  // namespace craft_planning
