
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

using freq_t = std::pair<bool, double>;

namespace
{
  typedef __int128 int128_t;
  typedef unsigned __int128 uint128_t;

  static constexpr double MINFREQ = 16000.0;

  hid_device* _device;

  struct TStatus
  {
    enum BIT : uint128_t
    {
      CONNECTED = ( 1 << 0 ),
      NEWSTATUS = ( 1 << 1 ),
      NEWCALIBRATION = ( 1 << 2 ),
      READCALIBRATION = ( 1 << 3 )
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

  std::pair<uint16_t, uint32_t> C{};  // first = C, second = L
  std::pair<uint16_t, uint32_t> L{};

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

  // async call result
  // true if valid
  static constexpr size_t M = 4;
  std::future<freq_t> last[ M ];
  size_t i{};
  double frequency;
}  // namespace

//static freq_t end( hid_device*, bool, double );
namespace
{
  inline void freq_( void )
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

    std::cout << "freq " << freq << std::endl;

    if ( freq <= MINFREQ ) return;
    frequency = freq;
    rb.put( frequency );
  }

  inline void _hid_ReadCalibration( void )
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

  inline void _hid_SendCalibration( void )
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

  using callback_t = void ( * )( uint8_t, double );
  void dummy( uint8_t, double ) {}
  callback_t callback{ &dummy };

  template <typename F, typename LC>
  inline double GetLC( F freq, LC lc )  // Возвращает значение емкости в pF
  {
    return ( ( 1.0 / ( 4.0 * ( M_PI * M_PI ) * ( freq * freq ) * lc ) ) * std::pow( 10, 21 ) );
  }

  inline void callback_call( void )
  {
    if ( hw_status & RELAY_BIT )
    {
      if ( reasonable( L.first ) && reasonable( L.second ) )
      {
        auto Lm = GetLC( frequency, L.first ) - L.second;
        callback( uint8_t{ 1 }, Lm );
      }
    }
    else
    {
      if ( reasonable( C.first ) && reasonable( C.second ) )
      {
        auto Cm = GetLC( frequency, C.second ) - C.first;
        callback( uint8_t{ 0 }, Cm );
      }
    }
  };
}  // namespace

/*
inline void clean( void )
{
  size_t j = ( i + 1 ) % M;
  if ( last[ j ].valid() )
  {
    std::cout << "valid " << j << std::endl;
    auto pair = last[ j ].get();
    if ( pair.first )
    {
      rb.put( pair.second );
      frequency = pair.second;

      callback_call();
    }
  }
}

inline void next( hid_device* dev )
{
  i = ( i + 1 ) % M;
  last[ i ] = std::async( std::launch::async, work, dev );
}

inline void if_connected_clean_next( hid_device* dev )
{
  if ( connected )
  {
    clean();
    next( dev );
  }
}  // namespace

static freq_t end( hid_device* dev, bool successfuly, double freq )
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
*/
void run( void )
{
  std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );

  if ( status.get<TStatus::BIT::NEWSTATUS>() )
  {
    _hid_SendStatus();
    //if_connected_clean_next( dev );
  }
  else if ( status.get<TStatus::BIT::CONNECTED>() && status.get<TStatus::BIT::READCALIBRATION>() )
  {
    _hid_ReadCalibration();
    //if_connected_clean_next( dev );
  }
  else if ( status.get<TStatus::BIT::CONNECTED>() && status.get<TStatus::BIT::NEWCALIBRATION>() )
  {
    //std::cout << "calibration" << std::endl;
    // auto pair = last.get();
    // if (pair.first) rb.put(pair.second);
    _hid_SendCalibration();

    //if_connected_clean_next( dev );
  }
  else if ( status.get<TStatus::BIT::CONNECTED>() )
  {
    freq_();
  }
  else if ( _device )
  {
    ::hid_close( _device );
    return;
  }
  std::cout << "exit" << std::endl;
  std::async( std::launch::async, run );
  //return ( std::make_pair( successfuly, freq ) );
}

/*bool init( void )
{
  connected = false;
  _device = _init();

  if ( _device )
  {
    next( _device );
    connected = true;
  }
  return ( connected );
}*/

bool init( void ( *callback_ )( uint8_t, double ) )
{
  callback = callback_;
  status.reset<TStatus::BIT::CONNECTED>();
  _device = _init();

  if ( _device )
  {
    std::async( std::launch::async, run );
    status.set<TStatus::BIT::CONNECTED>();
  }
  return ( status.get<TStatus::BIT::CONNECTED>() );
}

void deinit( void )
{
  hw_status &= ~PCPROGRAMRUN_BIT;
  hw_status &= ~RELAY_BIT;
  status.set<TStatus::BIT::NEWSTATUS>();
  status.reset<TStatus::BIT::CONNECTED>();
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

/*
double GetCapacitance( void )
{
  if ( !connected )
    if ( !init() ) return ( double{} );

  if ( status & RELAY_BIT ) set_relay_capicatance();

  if ( reasonable( C.first ) && reasonable( C.second ) )
  {
    return ( GetLC( frequency, C.second ) - C.first );
  }
  return ( double{} );
}

double GetInductance( void )
{
  if ( !connected )
    if ( !init() ) return ( double{} );

  if ( !( status & RELAY_BIT ) )
  {
    status |= RELAY_BIT;
    status |= NEWSTATUS_BIT;
  }

  if ( reasonable( L.first ) && reasonable( L.second ) )
  {
    return ( GetLC( frequency, L.first ) - L.second );
  }
  return ( double{} );
}
*/
