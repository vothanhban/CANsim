/*
 * CANMotorola.h
 *
 *  Created on: Nov 28, 2014
 *      Author: nvthanh
 */

#ifndef CANMOTOROLA_H_
#define CANMOTOROLA_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"{
#endif
double CANMotorolaGetFloat(const uint8_t source[], const uint8_t source_length,
        const uint16_t start_bit_position, const uint16_t bit_count,
        double factor,
        double offset
        );
bool CANMotorolaGetBool(const uint8_t source[], const uint8_t source_length,
        const uint16_t start_bit_position, const uint16_t bit_count
        );
double CANMotorolaSetFloat(double in_value, uint8_t destination[], const uint8_t destination_length,
        const uint16_t start_bit_position, const uint16_t bit_count,
        double factor,
        double offset
        );
bool CANMotorolaSetBool(bool in_value, uint8_t destination[], const uint8_t destination_length,   const uint16_t start_bit_position    );

#ifdef __cplusplus
}//extern "C"{
#endif

#endif /* CANMOTOROLA_H_ */
