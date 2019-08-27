/*
 * main.cpp
 *
 *  Created on: 9 ρεπο. 2019 π.
 *      Author: arsenal
 */
#include <chrono>
#include <iostream>
#include <thread>
#include <LCUsb.hpp>

int main( int, char** )
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
  while ( res )
    std::this_thread::sleep_for( std::chrono::milliseconds( 2000 ) );
  lc.deinit();
}
