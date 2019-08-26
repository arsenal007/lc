#pragma once

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

template <typename F>
struct Func
{
 private:
  F& func;

 public:
  template <typename A>
  auto operator()( A... a )
  {
    return ( func( a... ) );
  }

  Func( F& f ) : func{ f } {}
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
      callback_L( TLC::GetLC( freq, L.first ) - L.second );
    else if ( !status.get<TStatus::BIT::SC>() && !( hw_status & RELAY_BIT ) && valid_refC )
      callback_C( TLC::GetLC( freq, C.second ) - C.first );
  }

  double triggered_frequency_idle{};
  double triggered_frequency_ref{};

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

  template <typename T, size_t N>
  inline size_t sizeof_array( T ( & )[ N ] )
  {
    return ( N );
  }

  inline void calibrate( void )
  {
    _hid_ReadFrequency();
    double Ms = M - 1.0;
    auto fraction{ rb.size() / Ms };
    auto mean_frequency = rb.getM();
    auto standart_deviation = rb.getD();
    callback( std::make_tuple( fraction, mean_frequency, standart_deviation, tolerance * mean_frequency,
                               triggered_frequency_idle, triggered_frequency_ref ) );

    auto stable_freq{ rb.full() && ( standart_deviation < tolerance * mean_frequency ) };

    auto is_idle_not_triggered{ triggered_frequency_idle < MINFREQ };
    auto is_idle{ ( abs( mean_frequency - triggered_frequency_idle ) < 1000.0 ) && ( MINFREQ < mean_frequency ) };
    auto is_ref{ abs( mean_frequency - triggered_frequency_ref ) < 1000.0 };

    if ( stable_freq && ( is_idle_not_triggered || is_idle ) )
      triggered_frequency_idle = mean_frequency;
    else if ( !is_idle && stable_freq )
    {
      triggered_frequency_ref = mean_frequency;
      auto ref1 =
          floor( TLC::GetRef( triggered_frequency_idle, triggered_frequency_ref, customer_ref_lc, tolerance ) + 0.5 );
      if ( 100.0 < ref1 )
      {
        auto ref2 = floor( TLC::GetLC( triggered_frequency_idle, ref1 ) + 0.5 );
        //triggered_frequency_idle = double{};
        successfully_calibrated( ref1, ref2 );
      }
    }
  }

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

 public:
  inline bool init( void )
  {
    _device = ::hid_open( 0x16C0, 0x05DF, nullptr );
    if ( _device )
    {
      hw_status |= PCPROGRAMRUN_BIT;
      status.set<TStatus::BIT::NEWSTATUS>();
      status.set<TStatus::BIT::READCALIBRATION>();
      status.set<TStatus::BIT::C>();
      async();
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
    std::lock_guard<std::mutex> lockGuard{ mux };
    customer_ref_lc = Lc;
    tolerance = t;
    status.set<TStatus::BIT::SC>();
    hw_status |= RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();
    //std::cout << "Begin L Calibration. Calibration L: " << customer_ref_lc << "nH, tolerance: " << tolerance * 100.0 << "%" << std::endl;
  }

  template <typename C, typename T, typename F>
  inline void C_calibration( C Cl, T t, F func )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };
    customer_ref_lc = Cl;
    tolerance = t;
    status.set<TStatus::BIT::SC>();
    hw_status &= ~RELAY_BIT;
    status.set<TStatus::BIT::NEWSTATUS>();

    //func( std::make_pair( customer_ref_lc, tolerance * 100.0 ) );
    //std::cout << "Begin C Calibration. Calibration C: " << customer_ref_lc << "pF, tolerance: " << tolerance * 100.0 << "%" << std::endl;
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
