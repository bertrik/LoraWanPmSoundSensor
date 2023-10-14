#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "dnms.h"

DNMS::DNMS(Stream *serial, bool debug)
{
    _serial = serial;
    _debug = debug;
}

void DNMS::_printhex(const char *prefix, const uint8_t * buf, int len)
{
    if (_debug) {
        printf(prefix);
        for (int i = 0; i < len; i++) {
            printf("%02X", buf[i]);
        }
        printf("\n");
    }
}

bool DNMS::read_measurement(dnms_meas_t *meas)
{
    meas->LAMin = 21;
    meas->LAavg = 32;
    meas->LAMax = 43;
    return true;
}

