
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <future>
#include <iostream>
#include <tuple>
#include "TRingBuffer.hpp"
#include "hidapi.h"
//
#include "LCUsb.h"

using freq_t = std::pair<bool, double>;

namespace {

static constexpr double MINFREQ = 16000.0;

hid_device* _device;

struct TStatus {
  enum BIT : uint64_t {
    CONNECTED = (1 << 0),
    NEWSTATUS = (1 << 1),
    NEWCALIBRATION = (1 << 2),
    READCALIBRATION = (1 << 3),
    READCALIBRATION = (1 << 4)
  };

  TStatus(void) : status{} {}

  template <BIT b>
  inline bool get(void) {
    return (status & b != 0);
  }

  template <BIT b>
  inline void set(void) {
    status |= b;
  }

  template <BIT b>
  inline void reset(void) {
    status &= ~b;
  }

 private:
  uint64_t status;
};

template <typename A, typename B, typename C>
inline unsigned int _original_ReadIntValueFromBytes(A a, B b, C c) {
  return ((a << 16) | (b << 8) | c);
}

template <typename A, typename B>
inline uint16_t _original_ReadIntValueFromBytes(A a, B b) {
  return ((a << 8) | b);
}

template <typename T>
inline bool reasonable(T v) {
  return (std::numeric_limits<T>::min() + 10 < v) &&
         (v < std::numeric_limits<T>::max() - 10);
}

template <typename V, typename A, typename B, typename C>
inline bool _original_WriteIntValueToBytes(V value, A& a, B& b, C& c) {
  if (0xffffff < value) return (false);
  a = (value >> 16) & 0xFFFFFF;
  b = (value >> 8) & 0xFFFF;
  c = (value)&0xFF;
  return (true);
}

template <typename V, typename A, typename B>
inline bool _original_WriteIntValueToBytes(V value, A& a, B& b) {
  if (0xffff < value) return (false);
  a = (value >> 8) & 0xFFFF;
  b = (value)&0xFF;
  return (true);
}
uint8_t version{};

std::pair<uint16_t, uint32_t> C{};  // first = C, second = L
std::pair<uint16_t, uint32_t> L{};

static constexpr uint8_t RELAY_BIT = (1 << 7);
static constexpr uint8_t PCPROGRAMRUN_BIT = (1 << 6);
static constexpr uint8_t NEWSTATUS_BIT = (1 << 5);
static constexpr uint8_t NEWCALIBRATION_BIT = (1 << 4);
static constexpr uint8_t READCALIBRATION_BIT = (1 << 3);

unsigned char status;

inline void _hid_ProcessError(hid_device* dev) {
  if (dev) {
    ::hid_close(dev);
    dev = nullptr;
  }
}

inline auto _init(void) {
  auto dev = ::hid_open(0x16C0, 0x05DF, nullptr);

  status |= PCPROGRAMRUN_BIT;
  status &= ~RELAY_BIT;
  status |= NEWSTATUS_BIT;
  status |= READCALIBRATION_BIT;

  return (dev);
}  // namespace

// frequencies

Kernel::TRingBufferStatistic<double, 256> rb;

// async call result
// true if valid
static constexpr size_t M = 4;
std::future<freq_t> last[M];
size_t i{};
double frequency;
}  // namespace

static freq_t end(hid_device*, bool, double);

static freq_t work(hid_device* dev) {
  // if ( _needCalibration ) return;
  // TConnect hid{};
  // double freq = hid.ReadFrequency(dev);  //Принимаем данные
  if (dev == nullptr) {
    dev = _init();
    if (dev == nullptr) return (end(nullptr, false, double{}));
  }

#pragma pack(push, 1)
  struct report_t {
    unsigned char id;
    unsigned char data[3];
  };
#pragma pack(pop)

  report_t report;
  report.id = 0x01;

  int result =
      ::hid_get_feature_report(dev, (unsigned char*)&report, sizeof(report));

  if ((result == -1) || (dev == nullptr)) {
    std::cout << "result " << result << std::endl;
    _hid_ProcessError(dev);
    return (end(dev, false, double{}));
  }
  auto a = report.data[0];
  auto b = report.data[1];
  auto c = report.data[2];
  double freq = ((b << 8) + a) * 256 / 0.36 + c;
  std::cout << "freq " << freq << std::endl;
  if (freq <= MINFREQ) {
    // out of range
    return (end(dev, false, double{}));
  }
  return (end(dev, true, freq));
}

namespace {
bool connected{};

inline bool _hid_ReadCalibration(hid_device* dev) {
  if (dev == nullptr) {
    return (false);
  }

#pragma pack(push, 1)
  struct report_t {
    unsigned char id;
    unsigned char data[16];
  };
#pragma pack(pop)

  report_t g;
  g.id = 0x03;

  int result = ::hid_get_feature_report(_device, (unsigned char*)&g, sizeof(g));
  std::cout << "result" << result << std::endl;
  if (result == -1) {
    if (dev) ::hid_close(dev);
    dev = nullptr;
    return (dev);
  }

  // C calibration
  {
    auto refC = _original_ReadIntValueFromBytes(g.data[1], g.data[2]);
    auto refL =
        _original_ReadIntValueFromBytes(g.data[3], g.data[4], g.data[5]);
    std::cout << "refC " << refC << std::endl;
    std::cout << "refL " << refL << std::endl;
    if (reasonable(refC) && reasonable(refL)) C = std::make_pair(refC, refL);
  }

  // L calibration
  {
    auto refC = _original_ReadIntValueFromBytes(g.data[6], g.data[7]);
    auto refL =
        _original_ReadIntValueFromBytes(g.data[8], g.data[9], g.data[10]);
    std::cout << "refC " << refC << std::endl;
    std::cout << "refL " << refL << std::endl;

    if (reasonable(refC) && reasonable(refL)) L = std::make_pair(refC, refL);
  }

  return (true);
}

inline bool _hid_SendStatus(hid_device* dev) {
  std::cout << "_hid_SendStatus" << std::endl;
  if (dev == nullptr) {
    return (false);
  }

#pragma pack(push, 1)
  struct report_t {
    unsigned char id;
    unsigned char status;
  };
#pragma pack(pop)

  report_t report;
  report.id = 0x02;
  report.status = status;

  int result = ::hid_send_feature_report(
      _device, reinterpret_cast<uint8_t*>(&report), sizeof(report));

  if (result == -1) _hid_ProcessError(dev);
  return (true);
}

inline bool _hid_SendCalibration(hid_device* dev) {
#pragma pack(push, 1)
  struct report_t {
    unsigned char id;
    unsigned char data[16];
  };
#pragma pack(pop)

  report_t g;
  g.id = 0x03;
  g.data[0] = version;

  auto resonable_c_calibration = reasonable(C.first) && reasonable(C.second);
  if (resonable_c_calibration) {
    if (!_original_WriteIntValueToBytes(C.first, g.data[1], g.data[2]) ||
        !_original_WriteIntValueToBytes(C.second, g.data[3], g.data[4],
                                        g.data[5]))
      return (false);
  }

  auto resonable_l_calibration = reasonable(L.first) && reasonable(L.second);
  if (resonable_l_calibration) {
    if (!_original_WriteIntValueToBytes(L.first, g.data[6], g.data[7]) ||
        !_original_WriteIntValueToBytes(L.second, g.data[8], g.data[9],
                                        g.data[10]))
      return (false);
  }

  if (resonable_c_calibration || resonable_l_calibration) {
    int result =
        ::hid_send_feature_report(_device, (unsigned char*)&g, sizeof(g));
    if (result == -1) {
      _hid_ProcessError(dev);
      return (false);
    }
    return (true);
  }
  return (false);
}

using callback_t = void (*)(uint8_t, double);
void dummy(uint8_t, double) {}
callback_t callback{&dummy};

Kernel::TRingBufferStatistic<double, 256> _ref;

constexpr double pi = 3.1415926535897932385;

template <typename F, typename LC>
inline double GetLC(F freq, LC lc)  // Возвращает значение емкости в pF
{
  return ((1.0 / (4.0 * (pi * pi) * (freq * freq) * lc)) * std::pow(10, 21));
}

template <typename F1, typename F2, typename LC, typename T>
inline double GetRef(F1 freq1, F2 freq2, LC lc, T t) {
  double resultLC = (1.0 / (4.0 * (pi * pi) * lc)) *
                    (1.0 / (freq2 * freq2) - 1.0 / (freq1 * freq1)) *
                    std::pow(10.0, 21.0);
  _ref.put(resultLC);
  auto d = _ref.getD();
  auto m = _ref.getM();
  if (d < m * t) return (m);
  return (double{});
}

inline void callback_call(void) {
  if (status & RELAY_BIT) {
    if (reasonable(L.first) && reasonable(L.second)) {
      auto Lm = GetLC(frequency, L.first) - L.second;
      callback(uint8_t{1}, Lm);
    }
  } else {
    if (reasonable(C.first) && reasonable(C.second)) {
      auto Cm = GetLC(frequency, C.second) - C.first;
      callback(uint8_t{0}, Cm);
    }
  }
};

double triggered_frequency_idle{};
double triggered_frequency_lc{};
double customer_ref_lc{};
double tolerance;

inline bool is_idle_not_triggered(void) {
  return (MINFREQ > triggered_frequency_idle);
}

void calibrate(void) {
  if (is_idle_not_triggered()) {
    if (rb.full() && is_freq_stable()) {
      triggered_frequency_idle = get_stable_freq();
      callback(uint8_t{2}, triggered_frequency_idle);
    }
  } else if ((freq + 100.0) < triggered_frequency_idle) {
    auto ref1 = floor(
        GetRef(triggered_frequency_idle, freq, customer_ref_lc, tolerance) +
        0.5);
    if (100.0 < ref1) {
      callback(uint8_t{3}, freq);
      auto ref2 = floor(GetLC(triggered_frequency_idle, ref1) + 0.5);
      triggered_frequency_idle = double{};
      successfully_calibrated(ref1, ref2);
    }
  }
}  // namespace

}  // namespace

inline void clean(void) {
  size_t j = (i + 1) % M;
  if (last[j].valid()) {
    std::cout << "valid " << j << std::endl;
    auto pair = last[j].get();
    if (pair.first) {
      rb.put(pair.second);
      frequency = pair.second;

      callback_call();
    }
  }
}

inline void next(hid_device* dev) {
  i = (i + 1) % M;
  last[i] = std::async(std::launch::async, work, dev);
}

inline void if_connected_clean_next(hid_device* dev) {
  if (connected) {
    clean();
    next(dev);
  }
}  // namespace

static freq_t end(hid_device* dev, bool successfuly, double freq) {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (status & NEWSTATUS_BIT) {
    std::cout << "new status " << connected << std::endl;
    if (_hid_SendStatus(dev)) {
      std::cout << "_hid_SendStatus success" << std::endl;
      status &= ~NEWSTATUS_BIT;
    }
    if_connected_clean_next(dev);
  } else if (connected && (status & READCALIBRATION_BIT)) {
    if (_hid_ReadCalibration(dev)) {
      std::cout << "_hid_ReadCalibration success" << std::endl;
      status &= ~READCALIBRATION_BIT;
    }
    if_connected_clean_next(dev);
  } else if (connected && (status & NEWCALIBRATION_BIT)) {
    std::cout << "calibration" << std::endl;
    // auto pair = last.get();
    // if (pair.first) rb.put(pair.second);
    if (_hid_SendCalibration(dev)) {
      status &= NEWCALIBRATION_BIT;
      std::cout << "_hid_ReadCalibration success" << std::endl;
    }
    if_connected_clean_next(dev);
  } else if (connected) {
    clean();
    next(dev);
  } else if (dev)
    ::hid_close(dev);
  std::cout << "exit" << std::endl;
  return (std::make_pair(successfuly, freq));
}

void run(void) {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (status & NEWSTATUS_BIT) {
    std::cout << "new status " << connected << std::endl;
    if (_hid_SendStatus(dev)) {
      std::cout << "_hid_SendStatus success" << std::endl;
      status &= ~NEWSTATUS_BIT;
    }
    if_connected_clean_next(dev);
  } else if (connected && (status & READCALIBRATION_BIT)) {
    if (_hid_ReadCalibration(dev)) {
      std::cout << "_hid_ReadCalibration success" << std::endl;
      status &= ~READCALIBRATION_BIT;
    }
    if_connected_clean_next(dev);
  } else if (connected && (status & NEWCALIBRATION_BIT)) {
    std::cout << "calibration" << std::endl;
    // auto pair = last.get();
    // if (pair.first) rb.put(pair.second);
    if (_hid_SendCalibration(dev)) {
      status &= NEWCALIBRATION_BIT;
      std::cout << "_hid_ReadCalibration success" << std::endl;
    }
    if_connected_clean_next(dev);
  } else if (connected) {
    clean();
    next(dev);
  } else if (dev)
    ::hid_close(dev);
  std::cout << "exit" << std::endl;
  return (std::make_pair(successfuly, freq));
}

bool init(void) {
  connected = false;
  _device = _init();

  if (_device) {
    next(_device);
    connected = true;
  }
  return (connected);
}

bool init(void (*callback_)(uint8_t, double)) {
  callback = callback_;
  connected = false;
  _device = _init();

  if (_device) {
    next(_device);
    connected = true;
  }
  return (connected);
}

void deinit(void) {
  status &= ~PCPROGRAMRUN_BIT;
  status &= ~RELAY_BIT;
  status |= NEWSTATUS_BIT;
  connected = false;
}

double freq(void) { return (frequency); }

void set_relay_capicatance(void) {
  status &= ~RELAY_BIT;
  status |= NEWSTATUS_BIT;
};

double GetCapacitance(void) {
  if (!connected)
    if (!init()) return (double{});

  if (status & RELAY_BIT) set_relay_capicatance();

  if (reasonable(C.first) && reasonable(C.second)) {
    return (GetLC(frequency, C.second) - C.first);
  }
  return (double{});
}

double GetInductance(void) {
  if (!connected)
    if (!init()) return (double{});

  if (!(status & RELAY_BIT)) {
    status |= RELAY_BIT;
    status |= NEWSTATUS_BIT;
  }

  if (reasonable(L.first) && reasonable(L.second)) {
    return (GetLC(frequency, L.first) - L.second);
  }
  return (double{});
}
