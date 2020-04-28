#pragma once

/*
Convert Network bytes to host bytes
*/

#define __static_byteswap_16(x)  ((uint16_t)(			\
	(((uint16_t)(x) & (uint16_t)0x00FFU) << 8)		|	\
	(((uint16_t)(x) & (uint16_t)0xFF00U) >> 8)))

#define __static_byteswap_32(x) ((uint32_t)(			\
	(((uint32_t)(x) & (uint32_t)0x000000FFUL) << 24) |	\
	(((uint32_t)(x) & (uint32_t)0x0000FF00UL) <<  8) |	\
	(((uint32_t)(x) & (uint32_t)0x00FF0000UL) >>  8) |	\
	(((uint32_t)(x) & (uint32_t)0xFF000000UL) >> 24)))



inline uint16_t __bytes2uint16(uint16_t data16)
{
	unsigned char* bytes = reinterpret_cast<unsigned char*>(&data16);
	return (
		(((uint16_t)bytes[1]) << 8) | 
		 ((uint16_t)bytes[0])
	);
}


inline uint32_t __bytes2uint32(uint32_t data32)
{
	unsigned char* bytes = reinterpret_cast<unsigned char*>(&data32);
	return (
		(((uint32_t)bytes[3]) << 24) | 
		(((uint32_t)bytes[2]) << 16) |
		(((uint32_t)bytes[1]) <<  8) |
		 ((uint32_t)bytes[0])
	);
}