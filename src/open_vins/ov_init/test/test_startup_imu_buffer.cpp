#include "init/StartupImuBuffer.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

struct Sample { double timestamp; double value; };
using Buffer = ov_init::StartupImuBuffer<Sample>;

int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--replay") {
    Buffer buffer;
    double elapsed, stamp;
    while (std::cin >> elapsed >> stamp) {
      auto samples = buffer.push({stamp, elapsed}, 2.1);
      if (buffer.reset_reason()) std::cout << "reset " << elapsed << " " << buffer.reset_reason() << "\n";
      if (!samples.empty()) {
        assert(samples.back().timestamp - samples.front().timestamp >= 2.1);
        std::cout << "admitted " << elapsed << " count=" << samples.size() << " span="
                  << samples.back().timestamp - samples.front().timestamp << "\n";
        return 0;
      }
    }
    return 1;
  }
  for (double offset : {0.0, 0.7, -0.7}) {
    Buffer b;
    for (int i = 0; i <= 421; i++) {
      auto out = b.push({100.0 + offset + i * 0.005, double(i)}, 2.1);
      if (!out.empty()) {
        assert(out.size() >= 421);
        for (std::size_t j = 0; j < out.size(); j++) {
          assert(out[j].timestamp == 100.0 + offset + j * 0.005);
          assert(out[j].value == double(j));
        }
        break;
      }
    }
    assert(b.ready());
    auto out = b.push({104.0 + offset, 99.0}, 2.1);
    assert(out.size() == 1 && out.front().value == 99.0);
  }
  Buffer spike;
  spike.push({100.0, 1.0}, 2.1);
  assert(spike.push({110.0, 2.0}, 2.1).empty());
  assert(std::string(spike.reset_reason()) == "timestamp_gap");
  assert(spike.push({100.005, 3.0}, 2.1).empty());
  assert(std::string(spike.reset_reason()) == "timestamp_regression");
  assert(spike.size() == 1);
  for (int i = 1; i <= 422; i++) {
    auto out = spike.push({100.005 + i * 0.005, double(i)}, 2.1);
    if (!out.empty()) {
      assert(out.front().timestamp == 100.005);
      break;
    }
  }
  assert(spike.ready());
  Buffer dropout;
  dropout.push({100.0, 0.0}, 2.1);
  dropout.push({100.005, 0.0}, 2.1);
  assert(dropout.push({100.205, 0.0}, 2.1).empty());
  assert(dropout.size() == 1 && !dropout.ready());
  Buffer invalid;
  invalid.push({100.0, 0.0}, 2.1);
  assert(invalid.push({std::numeric_limits<double>::quiet_NaN(), 0.0}, 2.1).empty());
  assert(invalid.size() == 0 && !invalid.ready());
  Buffer duplicate;
  duplicate.push({100.0, 0.0}, 2.1);
  duplicate.push({100.0, 0.0}, 2.1);
  assert(duplicate.size() == 1 && !duplicate.ready());
  std::cout << "PASS: normal, constant offsets, unchanged samples, spike/rollback, dropout, invalid, duplicate, post-admission\n";
}
