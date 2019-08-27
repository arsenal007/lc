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
#include <functional>

#include <hidapi/hidapi.h>
#include "TRingBuffer.hpp"

struct TStatus
{
  enum BIT : uint64_t
  {
    CONNECTED = ( 1 << 0 ),  //CONNECTED
    NEW_STATUS = ( 1 << 1 ),
    NEW_CALIBRATION_SAVE = ( 1 << 2 ),
    READ_CALIBRATION = ( 1 << 3 ),
    RUN_CALIBRATION = ( 1 << 5 ),
    RUN_C_MEASURMENTS = ( 1 << 6 ),
    RUN_L_MEASURMENTS = ( 1 << 7 )
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
  uint64_t status;
};

using f_t = std::function<void( void )>;

struct TRun
{
 private:
  Kernel::TRingBuffer<f_t, 32> _rb;

 public:
  inline void operator()( void )
  {
    if ( !_rb.empty() )
    {
      auto f = _rb.get();
      f();
    }
  }

  inline void put( f_t item )
  {
    _rb.put( item );
  }

  TRun() : _rb{} {}
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

inline uint8_t version{};

static constexpr uint8_t RELAY_BIT = ( 1 << 7 );
static constexpr uint8_t PCPROGRAMRUN_BIT = ( 1 << 6 );
static constexpr uint8_t CALIBRATION_BIT = ( 1 << 5 );

inline uint8_t hw_status;
static constexpr size_t M = 1024;
inline std::mutex mux;
inline double customer_ref_lc, tolerance, measured;
inline f_t callback;
inline std::tuple<uint16_t, uint32_t, uint16_t, uint32_t> saved_ref;

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

struct TExecute : public TLC, TRun
{
 private:
  std::pair<uint16_t, uint32_t> C{};  // first = C, second = L
  std::pair<uint16_t, uint32_t> L{};
  Kernel::TRingBufferStatistic<double, M> rb;
  static constexpr double MINFREQ = 16000.0;

  double fraction, mean_frequency, standart_deviation, treshold, triggered_frequency_idle, triggered_frequency_ref;

  hid_device* _device{};

  static constexpr size_t Mt = 8;
  std::shared_future<void> fut[ Mt ]{};
  size_t k{};

  f_t _hid_SendStatus, _hid_ReadRef, _hid_SendRef, _hid_ReadFrequency;

  inline void _hid_ProcessError( void )
  {
    if ( _device )
    {
      ::hid_close( _device );
      _device = nullptr;
    }
  }

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
      put( _hid_SendRef );
    }
  }

  /*  template <typename T, size_t N>
  inline size_t sizeof_array( T ( & )[ N ] )
  {
    return ( N );
  }

    inline void calibrate( void )
  {
    _hid_ReadFrequency();
    double Ms = M - 1.0;
    fraction = rb.size() / Ms;
    mean_frequency = rb.getM();
    standart_deviation = rb.getD();
    treshold = tolerance * mean_frequency;
    callback();

    auto stable_freq{ rb.full() && ( standart_deviation < tolerance * mean_frequency ) };
    auto is_idle_not_triggered{ triggered_frequency_idle < MINFREQ };
    auto is_idle{ ( abs( mean_frequency - triggered_frequency_idle ) < MINFREQ ) && ( MINFREQ < mean_frequency ) };

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
*/
  inline void async_task_send( void )
  {
    if ( fut[ k ].valid() ) fut[ k ].get();
    fut[ k ] = std::async( std::launch::async, [&]( void ) {
      std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
      operator()();
      async_task_send();
    } );
    k = ( k + 1 ) % Mt;
  }

  /*void operator()( void )
  {
    std::this_thread::sleep_for( std::chrono::milliseconds( 33 ) );
    _run();
    if ( status.get<TStatus::BIT::NEW_STATUS>() )
      _hid_SendStatus();
    else if ( auto need_ref_get{ status.get<TStatus::BIT::CONNECTED>() &&
                                 status.get<TStatus::BIT::READ_CALIBRATION>() };
              need_ref_get )  //if CONNECTED && READ CALIBRATION bits - need to read calibration constants (refs)
      _hid_ReadRef();
    else if ( auto need_ref_set{ status.get<TStatus::BIT::C>() && status.get<TStatus::BIT::NEW_CALIBRATION>() };
              need_ref_set )
      _hid_SendRef();
    else if ( status.get<TStatus::BIT::RUN_CALIBRATION>() )
      calibrate();
    else if ( status.get<TStatus::BIT::C>() )
      _hid_ReadFrequency();
    else if ( _device )
    {
      ::hid_close( _device );
      return;
    }
    async();
  }*/

 public:
  TExecute( void )
      : _hid_SendStatus{ [&]( void ) {
          if ( _device == nullptr )
          {
            init();
            return;
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
        } },
        _hid_ReadRef{ [&]( void ) {
          if ( _device == nullptr )
          {
            init();
            return;
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

          saved_ref = std::make_tuple( _original_ReadIntValueFromBytes( g.data[ 1 ], g.data[ 2 ] ),
                                       _original_ReadIntValueFromBytes( g.data[ 3 ], g.data[ 4 ], g.data[ 5 ] ),
                                       _original_ReadIntValueFromBytes( g.data[ 6 ], g.data[ 7 ] ),
                                       _original_ReadIntValueFromBytes( g.data[ 8 ], g.data[ 9 ], g.data[ 10 ] ) );
          // C calibration
          {
            auto refC{ std::get<0>( saved_ref ) };
            auto refL{ std::get<1>( saved_ref ) };

            //std::cout << "C: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

            if ( reasonable( refC ) && reasonable( refL ) ) C = std::make_pair( refC, refL );
          }

          // L calibration
          {
            auto refC{ std::get<2>( saved_ref ) };
            auto refL{ std::get<3>( saved_ref ) };

            //std::cout << "L: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

            if ( reasonable( refC ) && reasonable( refL ) ) L = std::make_pair( refC, refL );
          }
          callback();
        } },
        _hid_SendRef{ [&]( void ) {
          if ( _device == nullptr )
          {
            init();
            return;
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
        } },
        _hid_ReadFrequency{ [&]( void ) {
          if ( _device == nullptr )
          {
            init();
            return;
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
          rb.put( freq );

          if ( auto measure_L{ ( reasonable( L.first ) && reasonable( L.second ) ) &&
                               ( hw_status & PCPROGRAMRUN_BIT ) && ( hw_status & RELAY_BIT ) &&
                               !( hw_status & CALIBRATION_BIT ) };
               measure_L )
          {
            measured = TLC::GetLC( freq, L.first ) - L.second;
            callback();
            put( _hid_ReadFrequency );
          }
          else if ( auto measure_C{ ( reasonable( C.first ) && reasonable( C.second ) ) &&
                                    ( hw_status & PCPROGRAMRUN_BIT ) && !( hw_status & RELAY_BIT ) &&
                                    !( hw_status & CALIBRATION_BIT ) };
                    measure_C )
          {
            measured = TLC::GetLC( freq, C.second ) - C.first;
            callback();
            put( _hid_ReadFrequency );
          }
          else if ( ( hw_status & PCPROGRAMRUN_BIT ) && ( hw_status & CALIBRATION_BIT ) )
          {
            double Ms = M - 1.0;
            fraction = rb.size() / Ms;
            mean_frequency = rb.getM();
            standart_deviation = rb.getD();
            treshold = tolerance * mean_frequency;
            callback();

            auto stable_freq{ rb.full() && ( standart_deviation < tolerance * mean_frequency ) };
            auto is_idle_not_triggered{ triggered_frequency_idle < MINFREQ };
            auto is_idle{ ( abs( mean_frequency - triggered_frequency_idle ) < MINFREQ ) &&
                          ( MINFREQ < mean_frequency ) };

            if ( stable_freq && ( is_idle_not_triggered || is_idle ) )
              triggered_frequency_idle = mean_frequency;
            else if ( !is_idle && stable_freq )
            {
              triggered_frequency_ref = mean_frequency;
              auto ref1 = floor(
                  TLC::GetRef( triggered_frequency_idle, triggered_frequency_ref, customer_ref_lc, tolerance ) + 0.5 );
              if ( 100.0 < ref1 )
              {
                auto ref2 = floor( TLC::GetLC( triggered_frequency_idle, ref1 ) + 0.5 );
                //triggered_frequency_idle = double{};
                successfully_calibrated( ref1, ref2 );
              }
            }
            put( _hid_ReadFrequency );
          }
          else if ( !( hw_status & PCPROGRAMRUN_BIT ) )
          {
            _hid_ProcessError();
          }
        } }

  {
  }

  inline bool init( void )
  {
    _device = ::hid_open( 0x16C0, 0x05DF, nullptr );
    if ( _device )
    {
      hw_status |= PCPROGRAMRUN_BIT;
      put( _hid_SendStatus );
      put( _hid_ReadRef );
      async_task_send();
      return ( true );
    }
    return ( false );
  }  // namespace

  template <typename F>
  inline bool init( F f )
  {
    callback = std::bind( f, saved_ref );
    return ( init() );
  }

  template <typename L, typename T, typename F>
  inline void L_calibration( L Lc, T t, F func )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };

    callback = std::bind( func, std::make_tuple( fraction, mean_frequency, standart_deviation, treshold,
                                                 triggered_frequency_idle, triggered_frequency_ref ) );
    customer_ref_lc = Lc;
    tolerance = t;
    hw_status |= RELAY_BIT;
    hw_status |= CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename C, typename T, typename F>
  inline void C_calibration( C Cl, T t, F func )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };

    callback = std::bind( func, std::make_tuple( fraction, mean_frequency, standart_deviation, treshold,
                                                 triggered_frequency_idle, triggered_frequency_ref ) );
    customer_ref_lc = Cl;
    tolerance = t;
    hw_status &= ~RELAY_BIT;
    hw_status |= CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename F>
  inline void C_measurments( F f )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };
    callback = std::bind( f, measured );
    hw_status &= ~RELAY_BIT;
    hw_status &= ~CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename F>
  inline void L_measurments( F f )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };
    callback = std::bind( f, measured );
    hw_status |= RELAY_BIT;
    hw_status &= ~CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  inline void deinit( void )
  {
    hw_status &= ~PCPROGRAMRUN_BIT;
    hw_status &= ~RELAY_BIT;
    hw_status &= ~CALIBRATION_BIT;
    put( _hid_SendStatus );
  }
};
