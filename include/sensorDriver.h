#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <stdint.h>

void sensorInit(void);
void sensorUpdate(void);
uint16_t getReflexCount(void);
void resetReflexCount(void);

#endif
