#ifndef TEST_F103_BME280_H
#define TEST_F103_BME280_H

bool BMP280_init();
bool BMP280_readMesure(int * a_dst_t, int * a_dst_p, int * a_dst_h);
bool BMP280_is_BME();

#endif // TEST_F103_BME280_H
