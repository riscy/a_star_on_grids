#include <string.h>
#include "benchmarks.h"
#include "test.h"

int main(int argc, char ** argv) {
  if (argc > 1 && strcmp(argv[1], "--test") == 0) {
    return test_path_costs();
  }
  benchmark_grid_costs();
  return 0;
}

