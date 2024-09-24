#ifndef HOME_SERVER_CRC32_H
#define HOME_SERVER_CRC32_H

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calculates 16 bits CRC value of given data.
 */
uint16_t crc16(const uint8_t *data, size_t data_length);
void cypher( uint8_t * a_data, size_t a_data_length, uint32_t a_key, uint32_t a_once );


#ifdef __cplusplus
}
#endif

#endif	// HOME_SERVER_CRC32_H
