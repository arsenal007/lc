/*
 * main.cpp
 *
 *  Created on: 9 ρεπο. 2019 π.
 *      Author: arsenal
 */
#include <chrono>
#include <iostream>
#include <thread>
#include "LCUsb.h"

void printout_capicatance(uint8_t state, double* value, size_t) {
  switch (state) {
    case (2): {
      std::cout.setf(std::ios::fixed);
      std::cout.setf(std::ios::showpoint);
      std::cout.precision(2);
      /*std::cout << "\r"
                << "                                        ";  // 40
      std::cout.flush();
      std::cout << "\r"
                << "                                        ";  // 40
      std::cout.flush();*/
      std::cout << "\r"
                << "B: " << value[0] * 100.0 << "%, F: " << value[1]
                << "Hz, D: " << value[2] << "Hz, T: " << value[3]
                << "Hz, F1: " << value[4] << "Hz, F2: " << value[5] << "Hz";
      std::cout.flush();
      break;
    }
    default: {
      std::cout << "state: " << uint32_t{state} << std::endl;
      double L = 89200.0;
      double tolerance = 0.000006;
      cal_L(L, tolerance);
    }
  }
}

int main(int argc, char** argv) {
  auto res = init(&printout_capicatance);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  double L = 89200.0;
  double tolerance = 0.000006;
  cal_L(L, tolerance);
  while (res) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  deinit();
};
