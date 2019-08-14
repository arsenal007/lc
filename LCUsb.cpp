
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <future>
#include <iostream>
#include <tuple>
#include "TRingBuffer.hpp"
#include "hidapi.h"
//
#include "LCUsb.h"

#include <cmath>

namespace
{
  typedef __int128 int128_t;
  typedef unsigned __int128 uint128_t;

  hid_device* _device;

  struct TStatus
  {
    enum BIT : uint128_t
    {
      C = ( 1 << 0 ),  //CONNECTED
      NEWSTATUS = ( 1 << 1 ),
      NEWCALIBRATION = ( 1 << 2 ),
      READCALIBRATION = ( 1 << 3 ),
      BSC = ( 1 << 4 )  //BEGINSIMPLECALIBARTION
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
  uint8_t hw_status;

  inline void _hid_ProcessError( void )
  {
    if ( _device )
    {
      ::hid_close( _device );
      _device = nullptr;
    }
  }

  inline auto _init( void )
  {
    auto dev = ::hid_open( 0x16C0, 0x05DF, nullptr );
    hw_status |= PCPROGRAMRUN_BIT;
    hw_status &= ~RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();
    status.set<TStatus::BIT::READCALIBRATION>();
    return ( dev );
  }  // namespace

  // frequencies

  Kernel::TRingBufferStatistic<double, 256> rb;
  double frequency;
}  // namespace

using callback_t = void ( * )( uint8_t, double );
void dummy( uint8_t, double ) {}
callback_t callback{ &dummy };

struct TExecute
{
 private:
  std::pair<uint16_t, uint32_t> C{};  // first = C, second = L
  std::pair<uint16_t, uint32_t> L{};

  static constexpr double MINFREQ = 16000.0;

  inline bool need_ref_get( void )
  {
    return ( status.get<TStatus::BIT::C>() && status.get<TStatus::BIT::READCALIBRATION>() );
  }

  inline bool need_ref_set( void )
  {
    return ( status.get<TStatus::BIT::C>() && status.get<TStatus::BIT::NEWCALIBRATION>() );
  }

  inline void _hid_SendStatus( void )
  {
    if ( _device == nullptr )
    {
      _device = _init();
      if ( _device == nullptr ) return;
    }

#pragma pack( push, 1 )
    struct report_t
    {
      unsigned char id;
      unsigned char status;
    };
#pragma pack( pop )

    report_t report;
    report.id = 0x02;
    report.status = hw_status;

    if ( ::hid_send_feature_report( _device, reinterpret_cast<uint8_t*>( &report ), sizeof( report ) ) == -1 )
      _hid_ProcessError();
    status.reset<TStatus::BIT::NEWSTATUS>();
  }

  inline void _hid_ReadRef( void )
  {
    if ( _device == nullptr )
    {
      _device = _init();
      if ( _device == nullptr ) return;
    }

#pragma pack( push, 1 )
    struct report_t
    {
      unsigned char id;
      unsigned char data[ 16 ];
    };
#pragma pack( pop )

    report_t g;
    g.id = 0x03;

    if ( ::hid_get_feature_report( _device, (unsigned char*)&g, sizeof( g ) ) == -1 )
    {
      _hid_ProcessError();
      return;
    }

    // C calibration
    {
      auto refC = _original_ReadIntValueFromBytes( g.data[ 1 ], g.data[ 2 ] );
      auto refL = _original_ReadIntValueFromBytes( g.data[ 3 ], g.data[ 4 ], g.data[ 5 ] );

      std::cout << "refC " << refC << std::endl;
      std::cout << "refL " << refL << std::endl;

      if ( reasonable( refC ) && reasonable( refL ) ) C = std::make_pair( refC, refL );
    }

    // L calibration
    {
      auto refC = _original_ReadIntValueFromBytes( g.data[ 6 ], g.data[ 7 ] );
      auto refL = _original_ReadIntValueFromBytes( g.data[ 8 ], g.data[ 9 ], g.data[ 10 ] );

      std::cout << "refC " << refC << std::endl;
      std::cout << "refL " << refL << std::endl;

      if ( reasonable( refC ) && reasonable( refL ) ) L = std::make_pair( refC, refL );
    }

    status.reset<TStatus::BIT::READCALIBRATION>();
  }

  inline void _hid_SendRef( void )
  {
    if ( _device == nullptr )
    {
      _device = _init();
      if ( _device == nullptr ) return;
    }

#pragma pack( push, 1 )
    struct report_t
    {
      unsigned char id;
      unsigned char data[ 16 ];
    };
#pragma pack( pop )

    report_t g;
    g.id = 0x03;
    g.data[ 0 ] = version;

    auto resonable_c_calibration = reasonable( C.first ) && reasonable( C.second );
    if ( resonable_c_calibration )
      if ( !_original_WriteIntValueToBytes( C.first, g.data[ 1 ], g.data[ 2 ] ) ||
           !_original_WriteIntValueToBytes( C.second, g.data[ 3 ], g.data[ 4 ], g.data[ 5 ] ) )
        return;

    auto resonable_l_calibration = reasonable( L.first ) && reasonable( L.second );
    if ( resonable_l_calibration )
      if ( !_original_WriteIntValueToBytes( L.first, g.data[ 6 ], g.data[ 7 ] ) ||
           !_original_WriteIntValueToBytes( L.second, g.data[ 8 ], g.data[ 9 ], g.data[ 10 ] ) )
        return;

    if ( resonable_c_calibration || resonable_l_calibration )
      if ( ::hid_send_feature_report( _device, (unsigned char*)&g, sizeof( g ) ) == -1 ) _hid_ProcessError();
    status.reset<TStatus::BIT::NEWCALIBRATION>();
  }

  inline void _hid_ReadFrequency( void )
  {
    if ( _device == nullptr )
    {
      _device = _init();
      if ( _device == nullptr ) return;
    }

#pragma pack( push, 1 )
    struct report_t
    {
      unsigned char id;
      unsigned char data[ 3 ];
    };
#pragma pack( pop )

    report_t report;
    report.id = 0x01;

    if ( ( ::hid_get_feature_report( _device, (unsigned char*)&report, sizeof( report ) ) == -1 ) )
    {
      _hid_ProcessError();
      return;
    }

    auto a = report.data[ 0 ];
    auto b = report.data[ 1 ];
    auto c = report.data[ 2 ];
    double freq = ( ( b << 8 ) + a ) * 256 / 0.36 + c;

    if ( freq <= MINFREQ ) return;
    frequency = freq;
    rb.put( frequency );
    auto valid_refL = reasonable( L.first ) && reasonable( L.second );
    auto valid_refC = reasonable( C.first ) && reasonable( C.second );
    if ( ( hw_status & RELAY_BIT ) && valid_refL )
    {
      auto Lm = GetLC( frequency, L.first ) - L.second;
      callback( uint8_t{ 1 }, Lm );
    }
    else if ( valid_refC )
    {
      auto Cm = GetLC( frequency, C.second ) - C.first;
      callback( uint8_t{ 0 }, Cm );
    }
  }
  Kernel::TRingBufferStatistic<double, 256> _ref;

  constexpr double pi = 3.1415926535897932385;

  template <typename F, typename LC>
  inline double GetLC( F freq, LC lc )  // Возвращает значение емкости в pF
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

 public:
  void operator()( void )
  {
    std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    if ( status.get<TStatus::BIT::NEWSTATUS>() )
      _hid_SendStatus();
    else if ( need_ref_get() )  //if CONNECTED && READ CALIBRATION bits - need to read calibration constants (refs)
      _hid_ReadRef();
    else if ( need_ref_set() )
      _hid_SendRef();
    else if ( status.get<TStatus::BIT::C>() )
      _hid_ReadFrequency();
    else if ( _device )
    {
      ::hid_close( _device );
      return;
    }
    std::async( std::launch::async, *this );
    //return ( std::make_pair( successfuly, freq ) );
  }
};

double triggered_frequency_idle{};
double triggered_frequency_lc{};
double customer_ref_lc{};
double tolerance;

inline bool is_idle_not_triggered( void )
{
  return ( MINFREQ > triggered_frequency_idle );
}

void calibrate( void )
{
  if ( is_idle_not_triggered() )
  {
    if ( rb.full() && is_freq_stable() )
    {
      triggered_frequency_idle = get_stable_freq();
      callback( uint8_t{ 2 }, triggered_frequency_idle );
    }
  }
  else if ( ( freq + 100.0 ) < triggered_frequency_idle )
  {
    auto ref1 = floor( GetRef( triggered_frequency_idle, freq, customer_ref_lc, tolerance ) + 0.5 );
    if ( 100.0 < ref1 )
    {
      callback( uint8_t{ 3 }, freq );
      auto ref2 = floor( GetLC( triggered_frequency_idle, ref1 ) + 0.5 );
      triggered_frequency_idle = double{};
      successfully_calibrated( ref1, ref2 );
    }
  }
}  // namespace

}  // namespace
>>>>>>> calibration

inline void _hid_ReadRefFrequency( void )
{
  if ( _device == nullptr )
  {
    _device = _init();
    if ( _device == nullptr ) return;
  }

#pragma pack( push, 1 )
  struct report_t
  {
    unsigned char id;
    unsigned char data[ 3 ];
  };
#pragma pack( pop )

  report_t report;
  report.id = 0x01;

  if ( ( ::hid_get_feature_report( _device, (unsigned char*)&report, sizeof( report ) ) == -1 ) )
  {
    _hid_ProcessError();
    return;
  }

  auto a = report.data[ 0 ];
  auto b = report.data[ 1 ];
  auto c = report.data[ 2 ];
  double freq = ( ( b << 8 ) + a ) * 256 / 0.36 + c;

  if ( freq <= MINFREQ ) return;
  frequency = freq;
  rb.put( frequency );

  auto valid_refL = reasonable( L.first ) && reasonable( L.second );

  auto valid_refC = reasonable( C.first ) && reasonable( C.second );

  if ( ( hw_status & RELAY_BIT ) && valid_refL )
  {
    auto Lm = GetLC( frequency, L.first ) - L.second;
    callback( uint8_t{ 1 }, Lm );
  }
  else if ( valid_refC )
  {
    auto Cm = GetLC( frequency, C.second ) - C.first;
    callback( uint8_t{ 0 }, Cm );
  }
<<<<<<< HEAD
}
== == == = if_connected_clean_next( dev );
}
else if ( connected )
{
  clean();
  next( dev );
}
else if ( dev )::hid_close( dev );
std::cout << "exit" << std::endl;
return ( std::make_pair( successfuly, freq ) );
}

void run( void )
{
  std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
  if ( status & NEWSTATUS_BIT )
  {
    std::cout << "new status " << connected << std::endl;
    if ( _hid_SendStatus( dev ) )
    {
      std::cout << "_hid_SendStatus success" << std::endl;
      status &= ~NEWSTATUS_BIT;
    }
    if_connected_clean_next( dev );
  }
  else if ( connected && ( status & READCALIBRATION_BIT ) )
  {
    if ( _hid_ReadCalibration( dev ) )
    {
      std::cout << "_hid_ReadCalibration success" << std::endl;
      status &= ~READCALIBRATION_BIT;
    }
    if_connected_clean_next( dev );
  }
  else if ( connected && ( status & NEWCALIBRATION_BIT ) )
  {
    std::cout << "calibration" << std::endl;
    // auto pair = last.get();
    // if (pair.first) rb.put(pair.second);
    if ( _hid_SendCalibration( dev ) )
    {
      status &= NEWCALIBRATION_BIT;
      std::cout << "_hid_ReadCalibration success" << std::endl;
    }
    if_connected_clean_next( dev );
  }
  else if ( connected )
  {
    clean();
    next( dev );
  }
  else if ( dev )
    ::hid_close( dev );
  std::cout << "exit" << std::endl;
  return ( std::make_pair( successfuly, freq ) );
}

bool init( void )
{
  connected = false;
  _device = _init();
>>>>>>> calibration
};

TExecute run;

bool init( void ( *callback_ )( uint8_t, double ) )
{
  callback = callback_;
  status.reset<TStatus::BIT::C>();
  _device = _init();
  if ( _device )
  {
    std::async( std::launch::async, run );
    status.set<TStatus::BIT::C>();
  }
  return ( status.get<TStatus::BIT::C>() );
}

void deinit( void )
{
  hw_status &= ~PCPROGRAMRUN_BIT;
  hw_status &= ~RELAY_BIT;
  status.set<TStatus::BIT::NEWSTATUS>();
  status.reset<TStatus::BIT::C>();
}

double freq( void )
{
  return ( frequency );
}

void set_relay_capicatance( void )
{
  hw_status &= ~RELAY_BIT;
  status.set<TStatus::BIT::NEWSTATUS>();
};
