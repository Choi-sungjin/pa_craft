// Request/response frame source for live tools: every byte read on stdin -> the newest aligned frame on stdout.
//   rs_stream [serial] [WxH@fps]      (default 640x360@30; depth 424x240 aligned to colour)
// stdout: once  "RSF1" w h (int32) fx fy cx cy depth_scale (float32); per request: colour BGR w*h*3, depth u16 w*h*2.
// Always the newest frame (older queued frames are dropped), so a slow consumer never lags behind.
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

static void put(const void * p, size_t n) {
  const char * c = static_cast<const char *>(p);
  while (n) {ssize_t k = write(1, c, n); if (k <= 0) _exit(0); c += k; n -= static_cast<size_t>(k);}
}

int main(int argc, char ** argv) try {
  const std::string serial = argc > 1 ? argv[1] : "";
  int w = 640, h = 360, fps = 30;
  if (argc > 2) std::sscanf(argv[2], "%dx%d@%d", &w, &h, &fps);
  rs2::config cfg; if (!serial.empty()) cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, fps);
  rs2::pipeline pipe; const auto prof = pipe.start(cfg);
  const float scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
  const auto in = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
  rs2::align align(RS2_STREAM_COLOR);
  for (int i = 0; i < 15; ++i) pipe.wait_for_frames();
  put("RSF1", 4); int32_t wh[2] = {in.width, in.height}; put(wh, sizeof(wh));
  float k[5] = {in.fx, in.fy, in.ppx, in.ppy, scale}; put(k, sizeof(k));
  char req;
  while (read(0, &req, 1) == 1) {
    rs2::frameset fs = pipe.wait_for_frames(), newer;
    while (pipe.poll_for_frames(&newer)) fs = newer;                   // drop the backlog
    fs = align.process(fs);
    put(fs.get_color_frame().get_data(), static_cast<size_t>(in.width) * in.height * 3);
    put(fs.get_depth_frame().get_data(), static_cast<size_t>(in.width) * in.height * 2);
  }
  pipe.stop();
  return 0;
} catch (const std::exception & e) {std::fprintf(stderr, "rs_stream: %s\n", e.what()); return 1;}
