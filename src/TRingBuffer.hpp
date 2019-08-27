#pragma once

#include <inttypes.h>
#include <cmath>
#include <type_traits>
#include <utility>

namespace Kernel
{
  template <typename T, typename Enable = void>
  class TSum;

  template <typename T>
  struct TSum<T, typename std::enable_if<std::is_integral<T>::value>::type>
  {
    using type = uint64_t;
  };

  template <typename T>
  struct TSum<T, typename std::enable_if<std::is_floating_point<T>::value>::type>
  {
    using type = double;
  };

  template <typename T>
  struct TRingBufferBase
  {
    using sum_t = typename TSum<T>::type;
    virtual sum_t getM( void ) = 0;
    virtual sum_t getD( void ) = 0;
    virtual void put( const T item ) = 0;
    virtual ~TRingBufferBase() {}
  };

  // https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
  // https://github.com/embeddedartistry/embedded-resources.git
  template <typename T, size_t size_>
  struct TRingBuffer
  {
    TRingBuffer( void ) : _buffer_{}, head_{}, tail_{} {}

    inline T get( void ) noexcept
    {
      if ( empty() )
      {
        return ( T() );
      }

      // Read data and advance the tail (we now have a free space)
      T val = _buffer_[ tail_ ];
      tail_ = ( tail_ + 1 ) % size_;
      return ( val );
    }

    inline bool empty( void ) const noexcept
    {
      // if head and tail are equal, we are empty
      return ( head_ == tail_ );
    }

    inline bool full( void ) const noexcept
    {
      // If tail is ahead the head by 1, we are full
      return ( ( ( head_ + 1 ) % size_ ) == tail_ );
    }

    size_t size( void ) const noexcept
    {
      return ( head_ > tail_ ? head_ - tail_ : size_ - ( tail_ - head_ ) );
    }

    inline void reset( void ) noexcept
    {
      head_ = tail_;
    }

    inline void put( const T item ) noexcept
    {
      _buffer_[ head_ ] = item;
      head_ = ( head_ + 1 ) % size_;
      if ( head_ == tail_ ) tail_ = ( tail_ + 1 ) % size_;
    }

   protected:
    T _buffer_[ size_ ];
    uint32_t head_;
    uint32_t tail_;
  };

  template <typename T, size_t K>
  struct TRingBufferWindow : public TRingBuffer<T, K>
  {
    TRingBufferWindow( void ) : TRingBuffer<T, K>{} {}

    template <size_t N>
    inline void view( T ( &buf )[ N ] )
    {
      auto M{ TRingBuffer<T, K>::size() };
      if ( M > N )
      {
        auto index = ( TRingBuffer<T, K>::head_ - 1 ) % TRingBuffer<T, K>::size_;
        auto i = N;
        do
        {
          index = ( index - 1 ) % TRingBuffer<T, K>::size_;
          i--;
          buf[ i ] = TRingBuffer<T, K>::_buffer_[ index ];
        } while ( i != 0 );
      }
    }
  };

  template <typename T>
  struct TRingBufferReadOnly
  {
    TRingBufferReadOnly( T* i ) : rb( i ) {}

    inline auto get( void )
    {
      return ( rb->get() );
    }

    inline auto empty( void )
    {
      return ( rb->empty() );
    }

    inline auto full( void )
    {
      return ( rb->full() );
    }

    inline auto size( void )
    {
      return ( rb->size() );
    }

   protected:
    T* rb;
  };

  template <typename T>
  struct TRingBufferWindowReadOnly : public TRingBufferReadOnly<T>
  {
    TRingBufferWindowReadOnly( T* i ) : TRingBufferReadOnly<T>( i ) {}

    template <size_t S>
    inline void view( T ( &buf )[ S ] )
    {
      TRingBufferReadOnly<T>::rb->view( buf );
    }
  };

  template <typename T, size_t size, typename S = typename TSum<T>::type>
  struct TRingBufferStatistic : public TRingBuffer<T, size>
  {
    TRingBufferStatistic( void ) : TRingBuffer<T, size>{} {}

    inline S getM( void ) noexcept
    {
      S sum{};
      auto N = TRingBuffer<T, size>::size();
      for ( size_t i = 0; i < N; i++ )
      {
        auto v = TRingBuffer<T, size>::get();
        TRingBuffer<T, size>::put( v );
        sum += v;
      }
      return ( sum / S{ N } );
    }

    inline S getD( void ) noexcept
    {
      auto N = TRingBuffer<T, size>::size();
      auto m = getM();
      S sum2{};
      for ( size_t i = 0; i < N; i++ )
      {
        auto v = TRingBuffer<T, size>::get();
        TRingBuffer<T, size>::put( v );
        auto a = ( v - m );
        sum2 += a * a;
      }
      return ( sqrt( sum2 / S{ N - S{ 1 } } ) );
    }
  };

}  // namespace Kernel
