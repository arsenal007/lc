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

#include "LCUsb.h"

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

using callback_t = void ( * )( uint8_t, double*, size_t );
void dummy( uint8_t, double*, size_t ) {}
callback_t callback{ &dummy };

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

struct TExecute : public TLC
{
 private:
  std::pair<uint16_t, uint32_t> C{};  // first = C, second = L
  std::pair<uint16_t, uint32_t> L{};

  Kernel::TRingBufferStatistic<double, M> rb;

  static constexpr double MINFREQ = 16000.0;

  inline bool need_ref_get( void )
  {
    return ( status.get<TStatus::BIT::C>() && status.get<TStatus::BIT::READCALIBRATION>() );
  }

  inline bool need_ref_set( void )
  {
    return ( status.get<TStatus::BIT::C>() && status.get<TStatus::BIT::NEWCALIBRATION>() );
  }

  inline void _hid_ProcessError( void )
  {
    if ( _device )
    {
      ::hid_close( _device );
      _device = nullptr;
    }
  }

  inline void _hid_SendStatus( void )
  {
    if ( _device == nullptr )
      if ( !init() ) return;

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
      if ( !init() ) return;

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

      std::cout << "C: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

      if ( reasonable( refC ) && reasonable( refL ) ) C = std::make_pair( refC, refL );
    }

    // L calibration
    {
      auto refC = _original_ReadIntValueFromBytes( g.data[ 6 ], g.data[ 7 ] );
      auto refL = _original_ReadIntValueFromBytes( g.data[ 8 ], g.data[ 9 ], g.data[ 10 ] );

      std::cout << "L: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

      if ( reasonable( refC ) && reasonable( refL ) ) L = std::make_pair( refC, refL );
    }

    status.reset<TStatus::BIT::READCALIBRATION>();
  }

  inline void _hid_SendRef( void )
  {
    if ( _device == nullptr )
      if ( !init() ) return;

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
      if ( !init() ) return;

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
    rb.put( freq );
    auto valid_refL = reasonable( L.first ) && reasonable( L.second );
    auto valid_refC = reasonable( C.first ) && reasonable( C.second );
    if ( !status.get<TStatus::BIT::SC>() && ( hw_status & RELAY_BIT ) && valid_refL )
    {
      auto Lm = TLC::GetLC( freq, L.first ) - L.second;
      callback( uint8_t{ 1 }, &Lm, 1u );
    }
    else if ( !status.get<TStatus::BIT::SC>() && !( hw_status & RELAY_BIT ) && valid_refC )
    {
      auto Cm = TLC::GetLC( freq, C.second ) - C.first;
      callback( uint8_t{ 0 }, &Cm, 1u );
    }
  }

  double triggered_frequency_idle{};
  double triggered_frequency_lc{};

  hid_device* _device{};

  static constexpr size_t Mt = 8;
  std::shared_future<void> fut[ Mt ]{};
  size_t k{};

  inline bool yesno( const std::string& prompt )
  {
    while ( true )
    {
      std::cout << prompt << " [y/n] ";

      std::string line;
      if ( !std::getline( std::cin, line ) )
      {
        std::cerr << "\n";
        std::cerr << "error: unexpected end of file\n";
        std::exit( EXIT_FAILURE );
      }

      std::transform( line.begin(), line.end(), line.begin(), []( unsigned char x ) { return std::tolower( x ); } );

      if ( line == "y" || line == "yes" )
      {
        return true;
      }
      if ( line == "n" || line == "no" )
      {
        return false;
      }
    }
  }

  template <typename R1, typename R2>
  void successfully_calibrated( R1 ref1, R2 ref2 )
  {
    status.reset<TStatus::BIT::SC>();
    if ( hw_status & RELAY_BIT )
    {
      L.first = floor( ref1 + 0.5 );
      L.second = floor( ref2 + 0.5 );
      std::cout << std::endl;
      std::cout << "refL: " << L.first << "pF, refL:" << L.second << "nH" << std::endl;
    }
    else
    {
      C.first = floor( ref2 + 0.5 );
      C.second = floor( ref1 + 0.5 );
      std::cout << std::endl;
      std::cout << "refC: " << C.first << "pF, refL:" << C.second << "nH" << std::endl;
    }

    if ( yesno( "save ref's? " ) )
    {
      status.set<TStatus::BIT::NEWCALIBRATION>();
    }
  }

  inline bool is_idle_not_triggered( void )
  {
    return ( MINFREQ > triggered_frequency_idle );
  }

  inline bool is_freq_stable( void )
  {
    return ( rb.getD() < tolerance * rb.getM() );
  }

  template <typename T, size_t N>
  size_t sizeof_array( T ( & )[ N ] )
  {
    return ( N );
  }

  inline void calibrate( void )
  {
    _hid_ReadFrequency();
    double Ms = M - 1.0;
    double p[] = { rb.size() / Ms,        rb.getM(), rb.getD(), tolerance * rb.getM(), triggered_frequency_idle,
                   triggered_frequency_lc };
    callback( uint8_t{ 2 }, p, sizeof_array( p ) );
    if ( is_idle_not_triggered() )
    {
      if ( rb.full() && is_freq_stable() )
      {
        triggered_frequency_idle = rb.getM();
      }
    }
    else if ( ( ( rb.getM() + 100.0 ) < triggered_frequency_idle ) && rb.full() && is_freq_stable() )
    {
      triggered_frequency_lc = rb.getM();
      auto ref1 =
          floor( TLC::GetRef( triggered_frequency_idle, triggered_frequency_lc, customer_ref_lc, tolerance ) + 0.5 );
      if ( 100.0 < ref1 )
      {
        auto ref2 = floor( TLC::GetLC( triggered_frequency_idle, ref1 ) + 0.5 );
        triggered_frequency_idle = double{};
        successfully_calibrated( ref1, ref2 );
      }
    }
  }

 public:
  inline void async( void )
  {
    if ( fut[ k ].valid() ) fut[ k ].get();
    fut[ k ] = std::async( std::launch::async, *this );
    k = ( k + 1 ) % Mt;
  }

  void operator()( void )
  {
    std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
    if ( status.get<TStatus::BIT::NEWSTATUS>() )
      _hid_SendStatus();
    else if ( need_ref_get() )  //if CONNECTED && READ CALIBRATION bits - need to read calibration constants (refs)
      _hid_ReadRef();
    else if ( need_ref_set() )
      _hid_SendRef();
    else if ( status.get<TStatus::BIT::SC>() )
      calibrate();
    else if ( status.get<TStatus::BIT::C>() )
      _hid_ReadFrequency();
    else if ( _device )
    {
      ::hid_close( _device );
      return;
    }
    async();
  }

  inline bool init( void )
  {
    _device = ::hid_open( 0x16C0, 0x05DF, nullptr );
    if ( _device )
    {
      hw_status |= PCPROGRAMRUN_BIT;
      hw_status &= ~RELAY_BIT;
      status.set<TStatus::BIT::NEWSTATUS>();
      status.set<TStatus::BIT::READCALIBRATION>();
      status.set<TStatus::BIT::C>();
    }
    else
    {
      status.reset<TStatus::BIT::C>();
    }
    return ( status.get<TStatus::BIT::C>() );
  }  // namespace

  template <typename F, typename T>
  inline void L_calibration( F Lc, T t )
  {
    customer_ref_lc = Lc;
    tolerance = t;
    status.set<TStatus::BIT::SC>();
    hw_status |= RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();
    std::cout << "Begin L Calibration. Calibration L: " << customer_ref_lc << "nH, tolerance: " << tolerance * 100.0
              << "%" << std::endl;
  }

  template <typename F, typename T>
  inline void C_calibration( F Cl, T t )
  {
    customer_ref_lc = Cl;
    tolerance = t;
    status.set<TStatus::BIT::SC>();
    hw_status &= ~RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();
    std::cout << "Begin C Calibration. Calibration C: " << customer_ref_lc << "pF, tolerance: " << tolerance * 100.0
              << "%" << std::endl;
  }

  inline void deinit( void )
  {
    hw_status &= ~PCPROGRAMRUN_BIT;
    hw_status &= ~RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();
    status.reset<TStatus::BIT::C>();
  }

  inline void relay_on( void )
  {
    hw_status |= RELAY_BIT;
  }

  inline void relay_off( void )
  {
    hw_status &= ~RELAY_BIT;
  }
};

TExecute run;

bool init( void ( *callback_ )( uint8_t, double*, size_t ) )
{
  callback = callback_;
  if ( run.init() ) run.async();
  return ( status.get<TStatus::BIT::C>() );
}

void cal_L( double L, double t )
{
  std::lock_guard<std::mutex> lockGuard{ mux };
  run.L_calibration( L, t );
}

void cal_C( double C, double t )
{
  std::lock_guard<std::mutex> lockGuard{ mux };
  run.C_calibration( C, t );
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
