#include <Arduino.h>

#ifndef __TYPES_H_
#define __TYPES_H_
// The hardware determines if the sensor is:
// a PM sensor with temp
// a dnms sensor with temp
typedef enum {
    E_SENSOR_UNDEFINED,
    E_SENSOR_PMSENSOR,
    E_SENSOR_DNMSSENSOR
} sensor_t;

typedef enum {
// a PM sensor SDS011
// a PM sensor SPS30
    E_PMSENSOR_NONE,
    E_PMSENSOR_SDS011,
    E_PMSENSOR_SPS30
} pmsensor_t;

typedef enum {
// a microphone ICS43434
// a microphone SPH0645
    E_DNMS_NONE,
    E_DNMS_ICS43434,
    E_DNMS_SPH0645
} dnmssensor_t;

#endif