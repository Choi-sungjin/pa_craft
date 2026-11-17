#include "executor.hpp"
#include <iostream>
int main(int argc, char **argv) {
  std::cout << std::unitbuf;
  std::cerr << std::unitbuf;
  puck::execute::Options opts;
  for (int i = 1; i < argc; ++i) opts.args.emplace_back(argv[i]);
  try { auto code = puck::execute::execute_main(opts); return puck::execute::is_interrupted() ? 130 : code; }
  catch (const std::exception &e) { std::cerr << "puck_execute: " << e.what() << '\n'; return 1; }
}
