#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>


int main() {
  std::cout << "start...\n";

  using namespace eeros;
  using namespace eeros::control;
  using namespace math;

  constexpr int A = 10;
  constexpr int B = 11;
  constexpr int M = 25000;
  constexpr int N = 2;
  constexpr int MN = M * N;
  constexpr int NO_OF_RUNS = 10000;

  Matrix<M, N> m1{};
  m1.zero();
  Matrix<M, N> gM1 = (m1 + 1) * A;
  Matrix<M, N> gM2 = (m1 + 1) * B;

  Gain<Matrix<M, N>, Matrix<M, N>, true> g1{gM1};
  g1.enable();

  Matrix<M, N> m2{};
  for (int i = 0; i < MN; i++) {
    m2[i] = i + 1;
  }

  Constant<Matrix<M, N>> c1{m2};

  g1.getIn().connect(c1.getOut());
  c1.run();

  int noOfFailedRuns{};

  static bool ready = false;

  std::thread t2{[&] {
    for (int i = 0; i < NO_OF_RUNS; i++) {

      while (!ready) {}
      g1.setGain(gM2);
    }
  }};

  // wait for t2.
  std::this_thread::sleep_for(std::chrono::nanoseconds{900000});

  for (int i = 0; i < NO_OF_RUNS; i++) {

    g1.setGain(gM1);
    ready = true;
    std::this_thread::sleep_for(std::chrono::nanoseconds{1055000});
    g1.run();
    ready = false;

    Matrix<M, N> res = g1.getOut().getSignal().getValue();

    for (int j = 0; j < MN; j++) {

      int expected = (j + 1) * A;
      int actual = res[j];

      if (expected != actual) {
        noOfFailedRuns++;

        // when the first is already wrong, t2 set the gain
        // before run was called.
        if (actual == B) break;

        // this should never be seen as long as the gain is
        // thread safe.
        std::cout << "wrong result. expected:" << expected <<
                  " actual:" << actual << " break run.\n";
        break;
      }
    }
  }

  // won't be zero since there can be failed runs even if gain is
  // thread safe.
  std::cout << "number of failed runs:" << noOfFailedRuns << "\n";

  ready = true; // so t2 can finish its execution.
  t2.join();

  std::cout << "done.\n";
}
