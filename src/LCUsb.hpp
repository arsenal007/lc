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

struct TRun
{
 private:
  using f_t = std::function<void( void )>;
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
  return ( ( std::numeric_limits<T>::min() + 100 < v ) && ( v < std::numeric_limits<T>::max() - 100 ) );
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
inline double customer_ref_lc;
inline double tolerance;

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
  std::pair<uint16_t, uint32_t> C{};           // first = C, second = L
  std::pair<uint16_t, uint32_t> L{};           // first = C, second = L
  Kernel::TRingBufferStatistic<double, M> rb;  // frequencies buffer
  static constexpr double MINFREQ = 16000.0;

  double fraction;
  double mean_frequency;
  double standart_deviation;
  double old_standart_deviation_idle;
  double old_standart_deviation_ref;
  double treshold;
  double triggered_frequency_idle;
  double triggered_frequency_ref;
  double measured;

  enum CSTAGE : uint8_t
  {
    FRACTION,
    IDLE_DIVARGANCE,
    REF_DIVARGANCE,
    FREQ,
    REF_C,
    REF_L
  };

  CSTAGE calibration_stage;

  std::tuple<uint16_t, uint32_t, uint16_t, uint32_t> saved_ref;
  hid_device* _device{};

  static constexpr size_t Mt = 8;
  size_t k{};
  std::shared_future<void> fut[ Mt ]{};

  template <typename T, typename SF = std::shared_future<T>>
  struct TSharedFuture : public SF
  {
    TSharedFuture() : first_call{ true } {}
    inline bool is_ready( void )
    {
      if ( auto ready( SF::wait_for( std::chrono::seconds( 0 ) ) == std::future_status::ready );
           SF::valid() && ready && first_call )
      {
        return ( true );
      }
      else
        return ( false );
    }

    template <typename Future>
    inline auto operator=( Future&& f )
    {
      first_call = true;
      return ( SF::operator=( std::forward<Future>( f ) ) );
    }

    inline const auto& get( void )
    {
      first_call = false;
      return ( SF::get() );
    }

   private:
    bool first_call;
  };

  TSharedFuture<bool> save_new_ref;

  std::function<void( void )> _hid_SendStatus;
  std::function<void( void )> _hid_ReadRef;
  std::function<void( void )> _hid_SendRef;
  std::function<void( void )> _hid_ReadFrequency;
  std::function<void( void )> callback_init;
  std::function<void( void )> callback_run;
  std::function<bool( void )> save;
  bool first_call;

  inline void _hid_ProcessError( void )
  {
    if ( _device )
    {
      ::hid_close( _device );
      _device = nullptr;
    }
  }

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
          {
            _hid_ProcessError();
            put( _hid_SendStatus );
          }
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
            put( _hid_ReadRef );
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

            //std::cout << "jjj C: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

            if ( reasonable( refC ) && reasonable( refL ) ) C = std::make_pair( refC, refL );
          }

          // L calibration
          {
            auto refC{ std::get<2>( saved_ref ) };
            auto refL{ std::get<3>( saved_ref ) };

            //std::cout << "jjj L: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;

            if ( reasonable( refC ) && reasonable( refL ) ) L = std::make_pair( refC, refL );
          }

          callback_init();
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
            if ( ::hid_send_feature_report( _device, (unsigned char*)&g, sizeof( g ) ) == -1 )
            {
              _hid_ProcessError();
              put( _hid_SendRef );
            }
        } },
        _hid_ReadFrequency{ [&]( void ) {
          if ( _device == nullptr )
          {
            if ( init() ) put( _hid_ReadFrequency );
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
            put( _hid_ReadFrequency );
            return;
          }
          auto a = report.data[ 0 ];
          auto b = report.data[ 1 ];
          auto c = report.data[ 2 ];
          double freq = ( ( b << 8 ) + a ) * 256 / 0.36 + c;
          if ( freq <= MINFREQ )
          {
            put( _hid_ReadFrequency );
            return;
          }
          rb.put( freq );
          if ( auto measure_L{ ( reasonable( L.first ) && reasonable( L.second ) ) &&
                               ( hw_status & PCPROGRAMRUN_BIT ) && ( hw_status & RELAY_BIT ) &&
                               !( hw_status & CALIBRATION_BIT ) };
               measure_L )
          {
            measured = TLC::GetLC( freq, L.first ) - L.second;
            callback_run();
            put( _hid_ReadFrequency );
          }
          else if ( auto measure_C{ ( reasonable( C.first ) && reasonable( C.second ) ) &&
                                    ( hw_status & PCPROGRAMRUN_BIT ) && !( hw_status & RELAY_BIT ) &&
                                    !( hw_status & CALIBRATION_BIT ) };
                    measure_C )
          {
            measured = TLC::GetLC( freq, C.second ) - C.first;
            callback_run();
            put( _hid_ReadFrequency );
          }
          else if ( ( hw_status & PCPROGRAMRUN_BIT ) && ( hw_status & CALIBRATION_BIT ) )
          {
            double Ms = M - 1.0;
            fraction = rb.size() / Ms;
            mean_frequency = rb.getM();
            standart_deviation = rb.getD();
            treshold = tolerance * mean_frequency;

            auto stable_freq{ rb.full() && ( standart_deviation < treshold ) };

            auto is_idle_not_triggered{ triggered_frequency_idle < MINFREQ };
            auto is_idle{ ( abs( mean_frequency - triggered_frequency_idle ) < 100.0 ) &&
                          ( MINFREQ < mean_frequency ) };

            auto is_ref_not_triggered{ triggered_frequency_ref < MINFREQ };
            auto is_ref{ ( abs( mean_frequency - triggered_frequency_ref ) < 100.0 ) && ( MINFREQ < mean_frequency ) };

            if ( stable_freq && ( is_idle_not_triggered || is_idle ) )
            {
              if ( standart_deviation < old_standart_deviation_idle )
              {
                triggered_frequency_idle = mean_frequency;
                old_standart_deviation_idle = standart_deviation;
                calibration_stage = CSTAGE::IDLE_DIVARGANCE;
                callback_run();
              }
              else
              {
                calibration_stage = CSTAGE::FREQ;
                callback_run();
              }
            }
            else if ( stable_freq && ( standart_deviation < old_standart_deviation_ref ) &&
                      ( is_ref_not_triggered || is_ref ) )
            {
              triggered_frequency_ref = mean_frequency;
              old_standart_deviation_ref = standart_deviation;
              calibration_stage = CSTAGE::REF_DIVARGANCE;
              callback_run();
            }
            else if ( auto ref1{ floor(
                          TLC::GetRef( triggered_frequency_idle, triggered_frequency_ref, customer_ref_lc, tolerance ) +
                          0.5 ) };
                      stable_freq && ( 100.0 < ref1 ) && is_ref )
            {
              auto ref2 = floor( TLC::GetLC( triggered_frequency_idle, ref1 ) + 0.5 );
              if ( first_call )
              {
                save_new_ref = std::async( std::launch::async, save );
                first_call = false;
              }
              else if ( save_new_ref.is_ready() )
              {
                if ( save_new_ref.get() ) put( _hid_SendRef );
                std::cout << std::endl << "succesfully calibrated" << std::endl;
                return;
              }
              if ( auto lc{ hw_status & RELAY_BIT }; lc )
              {
                L.first = floor( ref1 + 0.5 );
                L.second = floor( ref2 + 0.5 );
                calibration_stage = CSTAGE::REF_L;
                callback_run();
              }
              else
              {
                C.first = floor( ref2 + 0.5 );
                C.second = floor( ref1 + 0.5 );
                calibration_stage = CSTAGE::REF_C;
                callback_run();
              }
            }
            else if ( !rb.full() )
            {
              calibration_stage = CSTAGE::FRACTION;
              callback_run();
            }
            else
            {
              calibration_stage = CSTAGE::FREQ;
              callback_run();
            }
            put( _hid_ReadFrequency );
          }
          else if ( !( hw_status & PCPROGRAMRUN_BIT ) )
          {
            _hid_ProcessError();
          }
        } },
        first_call{ true }
  {
  }

  inline bool init( void )
  {
    _device = ::hid_open( 0x16C0, 0x05DF, nullptr );
    if ( _device )
    {
      hw_status |= PCPROGRAMRUN_BIT;
      put( _hid_SendStatus );
      return ( true );
    }
    return ( false );
  }  // namespace

  template <typename F>
  inline bool init( F f )
  {
    callback_init = std::bind( f, std::cref( saved_ref ) );
    auto success{ init() };
    if ( success )
    {
      put( _hid_ReadRef );
      async_task_send();
    }
    return ( success );
  }

  template <typename RefInductance, typename T, typename F, typename SaveQuestions>
  inline void L_calibration( RefInductance Lc, T t, F func, SaveQuestions s )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };

    hw_status |= RELAY_BIT;
    callback_run =
        std::bind( func, std::cref( calibration_stage ), std::cref( fraction ), std::cref( mean_frequency ),
                   std::cref( standart_deviation ), std::cref( treshold ), std::cref( triggered_frequency_idle ),
                   std::cref( triggered_frequency_ref ), std::cref( L.first ), std::cref( L.second ) );
    save = s;
    customer_ref_lc = Lc;
    tolerance = t;
    old_standart_deviation_idle = std::numeric_limits<double>::max();
    old_standart_deviation_ref = std::numeric_limits<double>::max();
    hw_status |= CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename RefCapacitance, typename T, typename F, typename SaveQuestions>
  inline void C_calibration( RefCapacitance Cl, T t, F func, SaveQuestions s )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };

    hw_status &= ~RELAY_BIT;
    callback_run =
        std::bind( func, std::cref( calibration_stage ), std::cref( fraction ), std::cref( mean_frequency ),
                   std::cref( standart_deviation ), std::cref( treshold ), std::cref( triggered_frequency_idle ),
                   std::cref( triggered_frequency_ref ), std::cref( C.first ), std::cref( C.second ) );
    save = s;
    customer_ref_lc = Cl;
    tolerance = t;
    old_standart_deviation_idle = std::numeric_limits<double>::max();
    old_standart_deviation_ref = std::numeric_limits<double>::max();
    hw_status |= CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename F>
  inline void C_measurments( F f )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };
    callback_run = std::bind( f, std::cref( measured ) );
    hw_status &= ~RELAY_BIT;
    hw_status &= ~CALIBRATION_BIT;
    put( _hid_SendStatus );
    put( _hid_ReadFrequency );
  }

  template <typename F>
  inline void L_measurments( F f )
  {
    std::lock_guard<std::mutex> lockGuard{ mux };
    callback_run = std::bind( f, std::cref( measured ) );
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
