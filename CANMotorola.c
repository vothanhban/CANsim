/*
 * CANMotorola.c
 *
 *  Created on: Nov 28, 2014
 *      Author: nvthanh
 */


#include "CANMotorola.h"
#include <common/Debug.h>
#include <stdio.h>
#include <string.h>
#include <endian.h>

//static const uint8_t reverse_mask[] =
//    { 0x55, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };
//static const uint8_t reverse_mask_xor[] =
//    { 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00 };

typedef union byteArray{
  uint64_t v64;
  unsigned char bytes[8];
}byteArray;
#define CHAR_BIT 8
static uint64_t get_bitfield(const uint8_t source[], const uint8_t source_length,
                const uint16_t offset, const uint16_t bit_count)
{
	byteArray value;
//	int i=0;
	uint16_t source_length_bits = source_length * CHAR_BIT;
	uint16_t new_offset = offset + (64 - source_length_bits);
//	DBG("\n");
	if(source_length > 8 || source_length_bits < (int)((int)offset/8 +1)*8) return 0;
	memset(value.bytes,0,sizeof(byteArray));
	memcpy(value.bytes+(8 - source_length),source,source_length);
	if(BYTE_ORDER == LITTLE_ENDIAN)
	{
		value.v64 = __builtin_bswap64(value.v64);
	}
	if( ((int)((int)new_offset/8 +1)*8 - ((int)bit_count + (int)( (int)new_offset%8))) < 0){
		printf("not enough bits\n");
		return 0;
	}

//	value.v64 = value.v64 << (new_offset  - bit_count + (8-new_offset%8));
//	value.v64 = value.v64 >> (new_offset  - bit_count + (8-new_offset%8));
// 	value.v64 = value.v64 >> (64 - (new_offset)   - (8-new_offset%8));
	uint16_t shift_left, shift_right;
	shift_left = ((new_offset/8 +1)*8 - (bit_count + ( new_offset%8)));
	shift_right =  64 - shift_left - bit_count;
	value.v64 = value.v64 << shift_left;
//	printf("<< %d\n",shift_left);
//	for (i=0;i<8;i++) printf("0x%X ", (int)value.bytes[i]);
//	printf("\n");

	value.v64 = value.v64 >> shift_left;
//	for (i=0;i<8;i++) printf("0x%X ", (int)value.bytes[i]);
//	printf("\n");
	value.v64 = value.v64 >> shift_right;
//	printf(">> %d\n",shift_right);
//	for (i=0;i<8;i++) printf("0x%X ", (int)value.bytes[i]);
//	printf("\n");
//	DBG("\n");
	return value.v64;
}

static bool set_bitfield(const uint64_t in_value, const uint16_t offset,
        const uint16_t bit_count, uint8_t destination[],
        uint16_t destination_length)
{
	bool ret=true;
	byteArray value;
	byteArray value_new ;
	value_new.v64  = in_value;
//	int i=0;
	uint16_t destination_length_bits = destination_length * CHAR_BIT;
	uint16_t new_offset = offset + (64 - destination_length_bits);
//	uint16_t shift_left, shift_right, left_pos, right_pos;
	if(destination_length > 8) return false;
	if(bit_count > destination_length_bits) return false;
	if((int ) 64 - (int)(new_offset/8 +1)*8  +  (int)(new_offset%8) < 0) return false;
	value_new.v64 <<= 64 - bit_count;
	value_new.v64 >>= 64 - bit_count;
	value_new.v64 <<= 64 - (new_offset/8 +1)*8  +  ( new_offset%8);
	memset(value.bytes,0,sizeof(byteArray));
	memcpy(value.bytes+(8 - destination_length),destination,destination_length);
	if(BYTE_ORDER == LITTLE_ENDIAN)
	{
		value.v64 = __builtin_bswap64(value.v64);
	}
	value.v64 |= value_new.v64;
	if(BYTE_ORDER == LITTLE_ENDIAN)
	{
		value.v64 = __builtin_bswap64(value.v64);
	}
	memcpy(destination,value.bytes+(8 - destination_length),destination_length);
	return ret;
}

inline double CANMotorolaGetFloat(const uint8_t source[], const uint8_t source_length,
        const uint16_t start_bit_position, const uint16_t bit_count,
        double factor,
        double offset
        )
{
	return (double)get_bitfield(source,source_length,start_bit_position,bit_count) * factor + offset;
}

inline bool CANMotorolaGetBool(const uint8_t source[], const uint8_t source_length,
        const uint16_t start_bit_position, const uint16_t bit_count
        )
{
	return (bool) (get_bitfield(source,source_length,start_bit_position,bit_count) == 1);
}


inline double CANMotorolaSetFloat(double in_value,uint8_t destination[], const uint8_t destination_length,
        const uint16_t start_bit_position, const uint16_t bit_count,
        double factor,
        double offset
        )
{
//	return (float)set_bitfield(source,source_length,start_bit_position,bit_count);
	if(set_bitfield((uint64_t)((in_value-offset)/factor),start_bit_position,bit_count,destination,destination_length) == true){
		return in_value;
	}
	return -1;
}
inline bool CANMotorolaSetBool(bool in_value,uint8_t destination[], const uint8_t destination_length,   const uint16_t start_bit_position    )
{
	if(set_bitfield((uint64_t)(in_value == true),start_bit_position,1,destination,destination_length) == true){
		return true;
	}
	return false;
}


