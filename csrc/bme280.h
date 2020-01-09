#ifndef __BME280__
#define __BME280__

int bme280_acquire(uint32_t* temp, uint32_t *hum, uint32_t *press);

#endif

