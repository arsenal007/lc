/*
 * LCUsb.h
 *
 *  Created on: 28 лип. 2019 р.
 *      Author: arsenal
 */

#ifndef LCUSB_H_
#define LCUSB_H_

#ifdef __cplusplus
extern "C"
{
#endif

  bool init( void );
  void set_callback_inductance( void ( * )( double ) );
  void set_callback_capicatance( void ( * )( double ) );
  void deinit( void );
  void set_relay_capicatance( void );
  void set_relay_inductance( void );
  void cal_L( double L, double t );
  void cal_C( double C, double t );

#ifdef __cplusplus
}
#endif

#endif /* LCUSB_H_ */
