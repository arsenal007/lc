#include "LCUsb.hpp"

#include <corecrt.h>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <algorithm>

#include "hidapi.h"
#include "TRingBuffer.hpp"

namespace
{
  typedef __int128 int128_t;
  typedef unsigned __int128 uint128_t;

}  // namespace

using callback_t = void ( * )( double );
void dummy( double ) {}
callback_t callback_L{ &dummy };
callback_t callback_C{ &dummy };
callback_t callback_full{ &dummy };
callback_t callback_mean_frequency{ &dummy };
callback_t callback_standart_deviation{ &dummy };
callback_t callback_treshold{ &dummy };
callback_t callback_frequency_idle{ &dummy };
callback_t callback_frequency_ref{ &dummy };

struct TLC
{
 private:
  Kernel::TRingBufferStatistic<double, M> _ref;

  static constexpr double pi = 3.1415926535897932385;

 public:
  template <typename F, typename LC>
  inline double GetLC( F freq, LC lc )
  {
    return ( ( 1.0 / ( 4.0 * ( pi * pi ) * ( freq * freq ) * lc ) ) * std::pow( 10, 21 ) );
  }

  template <typename F1, typename F2, typename LC, typename T>
  inline double GetRef( F1 freq1, F2 freq2, LC lc, T t )
  {
    double resultLC = ( 1.0 / ( 4.0 * ( pi * pi ) * lc ) ) * ( 1.0 / ( freq2 * freq2 ) - 1.0 / ( freq1 * freq1 ) ) *
                      std::pow( 10.0, 21.0 );
    _ref.put( resultLC );
    auto d = _ref.getD();
    auto m = _ref.getM();
    if ( d < m * t ) return ( m );
    return ( double{} );
  }
};

TExecute execute;

void set_callback_inductance( void ( *l )( double ) )
{
  callback_L = l;
}

void set_callback_capicatance( void ( *c )( double ) )
{
  callback_C = c;
}

void set_callback_full( void ( *c )( double ) )
{
  callback_full = c;
}

void set_callback_mean_frequency( void ( *c )( double ) )
{
  callback_mean_frequency = c;
}

callback_t callback_standart_deviation{ &dummy };
callback_t callback_treshold{ &dummy };
callback_t callback_frequency_idle{ &dummy };
callback_t callback_frequency_ref{ &dummy };

void cal_L( double L, double t )
{
  std::lock_guard<std::mutex> lockGuard{ mux };
  run.L_calibration( L, t );
}

void deinit( void )
{
  run.deinit();
}

void set_relay_capicatance( void )
{
  run.relay_off();
  status.set<TStatus::BIT::NEWSTATUS>();
};

void set_relay_inductance( void )
{
  run.relay_on();
  status.set<TStatus::BIT::NEWSTATUS>();
};
