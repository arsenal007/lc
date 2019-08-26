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

  struct TStatus
  {
    enum BIT : uint128_t
    {
      C = ( 1 << 0 ),  //CONNECTED
      NEWSTATUS = ( 1 << 1 ),
      NEWCALIBRATION = ( 1 << 2 ),
      READCALIBRATION = ( 1 << 3 ),
      SC = ( 1 << 5 )
    };

    TStatus( void ) : status{} {}

    template <BIT b>
    bool get( void )
    {
      return ( ( status & b ) != 0 );
    }

    template <BIT b>
    void set( void )
    {
      status |= b;
    }

    template <BIT b>
    void reset( void )
    {
      status &= ~b;
    }

   private:
    uint128_t status;
  };

  template <typename A, typename B, typename C>
  inline uint32_t _original_ReadIntValueFromBytes( A a, B b, C c )
  {
    return ( ( a << 16 ) | ( b << 8 ) | c );
  }

  template <typename A, typename B>
  inline uint16_t _original_ReadIntValueFromBytes( A a, B b )
  {
    return ( ( a << 8 ) | b );
  }

  template <typename T>
  inline bool reasonable( T v )
  {
    return ( ( std::numeric_limits<T>::min() + 10 < v ) && ( v < std::numeric_limits<T>::max() - 10 ) );
  }

  template <typename V, typename A, typename B, typename C>
  inline bool _original_WriteIntValueToBytes( V value, A& a, B& b, C& c )
  {
    if ( 0xffffff < value ) return ( false );
    a = ( value >> 16 ) & 0xFFFFFF;
    b = ( value >> 8 ) & 0xFFFF;
    c = (value)&0xFF;
    return ( true );
  }

  template <typename V, typename A, typename B>
  inline bool _original_WriteIntValueToBytes( V value, A& a, B& b )
  {
    if ( 0xffff < value ) return ( false );
    a = ( value >> 8 ) & 0xFFFF;
    b = (value)&0xFF;
    return ( true );
  }

  uint8_t version{};

  static constexpr uint8_t RELAY_BIT = ( 1 << 7 );
  static constexpr uint8_t PCPROGRAMRUN_BIT = ( 1 << 6 );

  TStatus status;
  double customer_ref_lc;
  double tolerance;
  uint8_t hw_status;
  static constexpr size_t M = 1024;
  std::mutex mux;
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

TExecute run;

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
