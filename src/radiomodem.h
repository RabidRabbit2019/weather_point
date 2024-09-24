// radiomodem.h
#ifndef METEO_RADIOMODEM_H
#define METEO_RADIOMODEM_H

#include <stdlib.h>
#include <stdint.h>

#define RM_BIT_RATE   9600


// инициализация радиомодема, вернёт 0 (ноль) в случае успеха
int rm_init();
// принять пакет
int rm_receive(uint8_t * a_dst, uint32_t timeout);
// отправить пакет
int rm_transmit(uint8_t* data, uint32_t len);
//
bool rm_decode_request( uint8_t * a_req, size_t a_req_len);


extern uint8_t g_rssi;
extern uint8_t g_packet_length;

#endif // METEO_RADIOMODEM_H
