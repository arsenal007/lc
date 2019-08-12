/*
 * LCUsb.h
 *
 *  Created on: 28 лип. 2019 р.
 *      Author: arsenal
 */

#ifndef LCUSB_H_
#define LCUSB_H_

#ifdef __cplusplus
extern "C" {
#endif

bool init(void (*callback_)(uint8_t, double));
void deinit(void);
double freq(void);
double GetCapacitance(void);
void set_relay_capicatance(void);

#ifdef __cplusplus
}
#endif

#endif /* LCUSB_H_ */
