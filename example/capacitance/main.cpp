/*
 * main.cpp
 *
 *  Created on: 9 ρεπο. 2019 π.
 *      Author: arsenal
 */
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include <vector>
#include <LCUsb.hpp>

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

class InputParser
{
 public:
  InputParser( int& argc, char** argv )
  {
    for ( int i = 1; i < argc; ++i )
      this->tokens.push_back( std::string( argv[ i ] ) );
  }
  /// @author iain
  const std::string& getCmdOption( const std::string& option ) const
  {
    std::vector<std::string>::const_iterator itr;
    itr = std::find( this->tokens.begin(), this->tokens.end(), option );
    if ( itr != this->tokens.end() && ++itr != this->tokens.end() )
    {
      return *itr;
    }
    static const std::string empty_string( "" );
    return empty_string;
  }
  /// @author iain
  bool cmdOptionExists( const std::string& option ) const
  {
    return std::find( this->tokens.begin(), this->tokens.end(), option ) != this->tokens.end();
  }

 private:
  std::vector<std::string> tokens;
};

int main( int argc, char** argv )
{
  TExecute lc;
  auto res = lc.init( []( auto t ) {
    {
      auto refC = std::get<0>( t );
      auto refL = std::get<1>( t );
      std::cout << "C: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;
    }
    {
      auto refC = std::get<2>( t );
      auto refL = std::get<3>( t );
      std::cout << "L: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;
    }
  } );
  std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
  InputParser input{ argc, argv };
  if ( input.cmdOptionExists( "-c" ) )
  {
    lc.C_measurments( []( auto Capicatance ) {
      std::string p{ " pF" };
      if ( Capicatance > 1000.0 )
      {
        p = " nF";
        Capicatance /= 1000.0;
        if ( Capicatance > 1000.0 )
        {
          p = " mF";
          Capicatance /= 1000.0;
        }
      }

      std::cout.setf( std::ios::fixed );
      std::cout.setf( std::ios::showpoint );
      std::cout.precision( 2 );
      std::cout << "\r";
      for ( size_t j = 0; j < 40; j++ )
        std::cout << " ";
      std::cout.flush();

      std::cout << "\rCapicatance: " << Capicatance << p;
      std::cout.flush();
    } );
  }
  else if ( input.cmdOptionExists( "-l" ) )
  {
    lc.L_measurments( []( auto Inducatance ) {
      std::string p{ " nH" };
      if ( Inducatance > 1000.0 )
      {
        p = " mkH";
        Inducatance /= 1000.0;
        if ( Inducatance > 1000.0 )
        {
          p = " mH";
          Inducatance /= 1000.0;
        }
      }

      std::cout.setf( std::ios::fixed );
      std::cout.setf( std::ios::showpoint );
      std::cout.precision( 2 );
      std::cout << "\r";
      for ( size_t j = 0; j < 40; j++ )
        std::cout << " ";
      std::cout.flush();

      std::cout << "\rInducatance: " << Inducatance << p;
      std::cout.flush();
    } );
  }
  else if ( input.cmdOptionExists( "-nc" ) )
  {
    bool first_call[ 4 ]{ true };
    lc.C_calibration(
        double{ 1000.0 }, double{ 0.000006 },
        [&]( auto pack ) {
          std::cout.setf( std::ios::fixed );
          std::cout.setf( std::ios::showpoint );
          std::cout.precision( 2 );

          auto stage{ std::get<0>( pack ) };

          if ( stage == 3 )
          {
            if ( first_call[ 0 ] )
            {
              first_call[ 0 ] = false;
              first_call[ 1 ] = true;
              first_call[ 2 ] = true;
              first_call[ 3 ] = true;
              std::cout << std::endl;
            }
            std::cout << "C: refC: " << std::get<7>( pack ) << "pF, refL: " << std::get<8>( pack ) << "nH" << std::endl;
          }
          if ( ( abs( std::get<5>( pack ) - std::get<2>( pack ) ) < 100 ) && ( stage == 1 ) )
          {
            if ( first_call[ 1 ] )
            {
              first_call[ 0 ] = true;
              first_call[ 1 ] = false;
              first_call[ 2 ] = true;
              std::cout << std::endl;
            }
            {
              std::cout.setf( std::ios::fixed );
              std::cout.setf( std::ios::showpoint );
              std::cout.precision( 2 );
              std::cout << "\r";
              for ( size_t j = 0; j < 40; j++ )
                std::cout << " ";
              std::cout << "\r";
              std::cout.flush();
            }

            std::cout << "\r"
                      << "F IDLE: " << std::get<4>( pack ) << "Hz, D: " << std::get<2>( pack )
                      << "Hz, TRESHOLD: " << std::get<3>( pack ) << "Hz";
          }
          else if ( ( abs( std::get<6>( pack ) - std::get<2>( pack ) ) < 100 ) && ( stage == 2 ) )
          {
            if ( first_call[ 2 ] )
            {
              first_call[ 0 ] = true;
              first_call[ 1 ] = true;
              first_call[ 2 ] = false;
              std::cout << std::endl;
            }
            {
              std::cout.setf( std::ios::fixed );
              std::cout.setf( std::ios::showpoint );
              std::cout.precision( 2 );
              std::cout << "\r";
              for ( size_t j = 0; j < 40; j++ )
                std::cout << " ";
              std::cout << "\r";
              std::cout.flush();
            }

            std::cout << "\r"
                      << "F REF: " << std::get<5>( pack ) << "Hz, D: " << std::get<2>( pack )
                      << "Hz, TRESHOLD: " << std::get<3>( pack ) << "Hz";
          }
          else if ( stage == 5 )
          {
            if ( first_call[ 0 ] )
            {
              first_call[ 0 ] = false;
              first_call[ 1 ] = true;
              first_call[ 2 ] = true;
              std::cout << std::endl;
            }
            {
              std::cout.setf( std::ios::fixed );
              std::cout.setf( std::ios::showpoint );
              std::cout.precision( 2 );
              std::cout << "\r";
              for ( size_t j = 0; j < 40; j++ )
                std::cout << " ";
              std::cout << "\r";
              std::cout.flush();
            }

            std::cout << "\r"
                      << "F: " << std::get<1>( pack ) << "Hz, D: " << std::get<2>( pack )
                      << "Hz, TRESHOLD: " << std::get<3>( pack ) << "Hz";
          }
          else if ( stage == 3 )
          {
            std::cout << std::endl
                      << "C: refC: " << std::get<7>( pack ) << "pF, refL: " << std::get<8>( pack ) << "nH" << std::endl;
          }
          else
          {
            {
              std::cout.setf( std::ios::fixed );
              std::cout.setf( std::ios::showpoint );
              std::cout.precision( 2 );
              std::cout << "\r";
              for ( size_t j = 0; j < 40; j++ )
                std::cout << " ";
              std::cout << "\r";
              std::cout.flush();
            }
            int barWidth = 40;
            std::cout << "[";
            int pos = barWidth * std::get<0>( pack );
            for ( int i = 0; i < barWidth; ++i )
            {
              if ( i < pos )
                std::cout << "=";
              else if ( i == pos )
                std::cout << ">";
              else
                std::cout << " ";
            }
            std::cout << "] " << p << " %\r";
            std::cout.flush();
          }

          std::cout.flush();
        },
        []( auto& refC, auto& refL ) {
          std::cout << std::endl << "C: refC: " << refC << "pF, refL: " << refL << "nH" << std::endl;
        },
        []() -> bool {
          std::cout << std::endl << "save ref's? [y/n]";

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
        } );
  }
  while ( res )
    std::this_thread::sleep_for( std::chrono::milliseconds( 2000 ) );
  lc.deinit();
}
