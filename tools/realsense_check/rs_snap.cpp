// Grab one aligned RealSense frame: colour (BGR) + depth aligned to colour, both at the colour resolution.
//   rs_snap <out_prefix> [width height] [serial]   (default 640 360, the craft_sensors camera.yaml colour mode)
// Writes <prefix>_color.ppm, <prefix>_depth.u16 (little-endian, row-major), <prefix>_meta.txt
// (width height fx fy ppx ppy depth_scale serial). 30 warm-up frames for auto exposure.
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char ** argv) try {
  if (argc < 2) {std::cerr << "usage: rs_snap <out_prefix> [width height] [serial]\n"; return 2;}
  const std::string out = argv[1];
  const int w = argc > 3 ? std::atoi(argv[2]) : 640, h = argc > 3 ? std::atoi(argv[3]) : 360;
  rs2::config cfg;
  if (argc > 4) {cfg.enable_device(argv[4]);}
  cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);
  rs2::pipeline pipe;
  const auto profile = pipe.start(cfg);
  const float scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
  const std::string serial = profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  rs2::align align(RS2_STREAM_COLOR);
  rs2::frameset frames;
  for (int i = 0; i < 30; ++i) {frames = pipe.wait_for_frames();}
  frames = align.process(pipe.wait_for_frames());
  const auto color = frames.get_color_frame();
  const auto depth = frames.get_depth_frame();
  const auto in = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
  std::ofstream ppm(out + "_color.ppm", std::ios::binary);
  ppm << "P6\n" << in.width << " " << in.height << "\n255\n";
  const auto * bgr = static_cast<const uint8_t *>(color.get_data());
  for (int i = 0; i < in.width * in.height; ++i) {ppm.put(bgr[3 * i + 2]); ppm.put(bgr[3 * i + 1]); ppm.put(bgr[3 * i]);}
  std::ofstream(out + "_depth.u16", std::ios::binary).write(static_cast<const char *>(depth.get_data()),
    static_cast<std::streamsize>(depth.get_width()) * depth.get_height() * 2);
  std::ofstream(out + "_meta.txt") << in.width << " " << in.height << " " << in.fx << " " << in.fy << " " << in.ppx
    << " " << in.ppy << " " << scale << " " << serial << "\n";
  pipe.stop();
  std::cout << "saved " << out << " (" << in.width << "x" << in.height << ", fx " << in.fx << ", serial " << serial << ")\n";
  return 0;
} catch (const rs2::error & e) {
  std::cerr << "realsense: " << e.what() << " (" << e.get_failed_function() << ")\n"; return 1;
}
