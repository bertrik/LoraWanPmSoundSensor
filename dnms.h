#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float LAMin;
    float LAMax;
    float LAavg;
    uint16_t id;
} dnms_meas_t;

class DNMS {

private:
    Stream *_serial;
    bool _debug;
    void _printhex(const char *prefix, const uint8_t * buf, int len);

public:
    explicit DNMS(Stream *serial, bool debug = false);
    bool read_measurement(dnms_meas_t *measuremnt);

};

