// Live RealSense rendering stress test: sweep stream modes, render colour | depth side by side (OpenCV window),
// overlay and log camera fps, render fps, dropped frames and per-frame work.
//   rs_live [serial] [seconds_per_mode]   (DISPLAY must point at the desktop; q / Esc ends)
// Per frame (as the perception path): align depth to colour (when both), colourise depth, compose, imshow.
// Writes out/live_sweep.csv.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

struct Mode {int cw, ch, cfps, dw, dh, dfps; bool align; const char * name;};
using Clock = std::chrono::steady_clock;
static double ms(Clock::duration d) {return std::chrono::duration<double, std::milli>(d).count();}

int main(int argc, char ** argv) try {
  const std::string serial = argc > 1 ? argv[1] : "";
  const double seconds = argc > 2 ? std::atof(argv[2]) : 8.0;
  const std::vector<Mode> modes = {
    {640, 360, 30, 424, 240, 30, true, "current config  C640x360@30 + D424x240@30, aligned"},
    {640, 360, 60, 424, 240, 60, true, "C640x360@60 + D424x240@60, aligned"},
    {640, 360, 60, 424, 240, 90, true, "C640x360@60 + D424x240@90, aligned"},
    {848, 480, 60, 848, 480, 90, true, "C848x480@60 + D848x480@90, aligned"},
    {1280, 720, 30, 1280, 720, 30, true, "C1280x720@30 + D1280x720@30, aligned"},
    {640, 360, 60, 0, 0, 0, false, "colour only C640x360@60"},
    {0, 0, 0, 848, 480, 90, false, "depth only D848x480@90"},
    {0, 0, 0, 848, 100, 300, false, "depth only D848x100@300"},
  };
  std::ofstream csv("out/live_sweep.csv");
  csv << "mode,cam_color_fps,cam_depth_fps,render_fps,color_drop_pct,depth_drop_pct,proc_ms_avg,proc_ms_p95,show_ms_avg\n";
  cv::namedWindow("rs_live", cv::WINDOW_AUTOSIZE);
  bool quit = false;
  for (const auto & m : modes) {
    if (quit) break;
    rs2::config cfg;
    if (!serial.empty()) cfg.enable_device(serial);
    if (m.cw) cfg.enable_stream(RS2_STREAM_COLOR, m.cw, m.ch, RS2_FORMAT_BGR8, m.cfps);
    if (m.dw) cfg.enable_stream(RS2_STREAM_DEPTH, m.dw, m.dh, RS2_FORMAT_Z16, m.dfps);
    rs2::pipeline pipe;
    try {pipe.start(cfg);} catch (const rs2::error & e) {std::printf("%-52s  cannot start: %s\n", m.name, e.what()); continue;}
    rs2::align align(RS2_STREAM_COLOR);
    for (int i = 0; i < 15; ++i) pipe.wait_for_frames();   // auto exposure settles
    unsigned long long c_first = 0, c_last = 0, d_first = 0, d_last = 0, c_n = 0, d_n = 0;
    std::vector<double> proc; double show_sum = 0; int renders = 0;
    const auto t0 = Clock::now(); auto last_hud = t0; double hud_cam = 0, hud_render = 0; int hud_renders = 0;
    unsigned long long hud_c0 = 0;
    while (ms(Clock::now() - t0) < seconds * 1000) {
      rs2::frameset fs;
      if (!pipe.poll_for_frames(&fs)) {if ((cv::waitKey(1) & 0xff) == 'q') {quit = true; break;} continue;}
      const auto a = Clock::now();
      if (auto cf = fs.get_color_frame()) {const auto n = cf.get_frame_number(); if (!c_n) c_first = n; c_last = n; ++c_n;}
      if (auto df = fs.get_depth_frame()) {const auto n = df.get_frame_number(); if (!d_n) d_first = n; d_last = n; ++d_n;}
      if (m.align && m.cw && m.dw) fs = align.process(fs);
      cv::Mat left, right;
      if (auto cf = fs.get_color_frame())
        left = cv::Mat(cf.get_height(), cf.get_width(), CV_8UC3, const_cast<void *>(cf.get_data())).clone();
      if (auto df = fs.get_depth_frame()) {
        cv::Mat d16(df.get_height(), df.get_width(), CV_16UC1, const_cast<void *>(df.get_data())), d8;
        d16.convertTo(d8, CV_8U, 255.0 / 1500.0);             // 0-1.5 m
        cv::applyColorMap(d8, right, cv::COLORMAP_TURBO);
      }
      cv::Mat view;
      if (!left.empty() && !right.empty()) {
        if (right.rows != left.rows) cv::resize(right, right, cv::Size(right.cols * left.rows / right.rows, left.rows));
        cv::hconcat(left, right, view);
      } else view = left.empty() ? right : left;
      if (view.cols > 1700) cv::resize(view, view, cv::Size(1700, view.rows * 1700 / view.cols));
      proc.push_back(ms(Clock::now() - a));
      const auto now = Clock::now();
      if (ms(now - last_hud) >= 500) {
        const double dt = ms(now - last_hud) / 1000;
        hud_cam = (m.cw ? (c_last - hud_c0) : d_n) / dt; hud_render = hud_renders / dt;
        hud_c0 = m.cw ? c_last : 0; if (!m.cw) d_n = d_n; last_hud = now; hud_renders = 0;
      }
      char hud[256];
      std::snprintf(hud, sizeof(hud), "%s", m.name);
      cv::putText(view, hud, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 0}, 4);
      cv::putText(view, hud, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1);
      std::snprintf(hud, sizeof(hud), "render %.0f fps   work %.1f ms   t %.0f/%.0f s", hud_render, proc.back(),
                    ms(now - t0) / 1000, seconds);
      cv::putText(view, hud, {10, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 0}, 4);
      cv::putText(view, hud, {10, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
      const auto s = Clock::now();
      cv::imshow("rs_live", view);
      const int key = cv::waitKey(1) & 0xff;
      show_sum += ms(Clock::now() - s); ++renders; ++hud_renders;
      if (key == 'q' || key == 27) {quit = true; break;}
    }
    const double el = ms(Clock::now() - t0) / 1000;
    pipe.stop();
    auto rate = [&](unsigned long long f, unsigned long long l, unsigned long long n) {return n > 1 ? (n - 1) / el : 0.0;};
    auto drop = [](unsigned long long f, unsigned long long l, unsigned long long n) {
      return n > 1 ? 100.0 * (1.0 - double(n - 1) / double(l - f)) : 0.0;};
    std::sort(proc.begin(), proc.end());
    double avg = 0; for (double p : proc) avg += p; avg = proc.empty() ? 0 : avg / proc.size();
    const double p95 = proc.empty() ? 0 : proc[proc.size() * 95 / 100];
    const double cfps = rate(c_first, c_last, c_n), dfps = rate(d_first, d_last, d_n), rfps = renders / el;
    const double cdrop = drop(c_first, c_last, c_n), ddrop = drop(d_first, d_last, d_n);
    std::printf("%-52s  cam C %5.1f D %5.1f fps | render %5.1f fps | drop C %4.1f%% D %4.1f%% | work %.1f ms (p95 %.1f) imshow %.1f ms\n",
                m.name, cfps, dfps, rfps, cdrop, ddrop, avg, p95, renders ? show_sum / renders : 0.0);
    std::fflush(stdout);
    csv << '"' << m.name << "\"," << cfps << ',' << dfps << ',' << rfps << ',' << cdrop << ',' << ddrop << ',' << avg << ','
        << p95 << ',' << (renders ? show_sum / renders : 0.0) << '\n';
  }
  cv::destroyAllWindows();
  return 0;
} catch (const std::exception & e) {std::fprintf(stderr, "error: %s\n", e.what()); return 1;}
