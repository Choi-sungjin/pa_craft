// Live depth map only: rs_depth [serial] [WxH@fps] [max_m]   (default 848x480@90, 0-1.5 m; q / Esc closes)
// TURBO colour map (near blue -> far red), HUD: fps, centre-pixel distance and the scale bar.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <string>

int main(int argc, char ** argv) try {
  const std::string serial = argc > 1 ? argv[1] : "";
  int w = 848, h = 480, fps = 90; double max_m = argc > 3 ? std::atof(argv[3]) : 1.5;
  if (argc > 2) std::sscanf(argv[2], "%dx%d@%d", &w, &h, &fps);
  rs2::config cfg; if (!serial.empty()) cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
  rs2::pipeline pipe; const auto prof = pipe.start(cfg);
  const float scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
  cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
  auto last = std::chrono::steady_clock::now(); int n = 0; double shown = 0;
  while (true) {
    rs2::frameset fs = pipe.wait_for_frames();
    auto df = fs.get_depth_frame();
    cv::Mat d16(df.get_height(), df.get_width(), CV_16UC1, const_cast<void *>(df.get_data())), d8, view;
    d16.convertTo(d8, CV_8U, 255.0 / (max_m / scale));
    cv::applyColorMap(d8, view, cv::COLORMAP_TURBO);
    view.setTo(cv::Scalar(0, 0, 0), d16 == 0);                       // no depth = black
    if (view.cols < 800) cv::resize(view, view, cv::Size(), 2, 2, cv::INTER_NEAREST);
    const float c = df.get_distance(df.get_width() / 2, df.get_height() / 2);
    ++n; const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    if (dt >= 0.5) {shown = n / dt; n = 0; last = now;}
    char t[160];
    std::snprintf(t, sizeof(t), "depth %dx%d@%d  %.0f fps   centre %.3f m   black = no depth", w, h, fps, shown, c);
    cv::putText(view, t, {10, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 0, 0}, 4);
    cv::putText(view, t, {10, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {255, 255, 255}, 1);
    cv::drawMarker(view, {view.cols / 2, view.rows / 2}, {255, 255, 255}, cv::MARKER_CROSS, 22, 2);
    for (int i = 0; i < 256; ++i) {                                     // scale bar 0 .. max_m
      cv::Mat px(1, 1, CV_8U, cv::Scalar(i)), col; cv::applyColorMap(px, col, cv::COLORMAP_TURBO);
      cv::line(view, {10 + i, view.rows - 30}, {10 + i, view.rows - 14}, cv::Scalar(col.at<cv::Vec3b>(0)), 1);
    }
    std::snprintf(t, sizeof(t), "0 m%*s%.1f m", 36, "", max_m);
    cv::putText(view, t, {10, view.rows - 36}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
    cv::imshow("depth", view);
    const int k = cv::waitKey(1) & 0xff;
    // Only q / Esc end it: on this desktop (GTK + mutter) WND_PROP_VISIBLE reads 0 while the window is shown.
    if (k == 'q' || k == 27) break;
  }
  pipe.stop();
  return 0;
} catch (const std::exception & e) {std::fprintf(stderr, "error: %s\n", e.what()); return 1;}
