#include <Arduino.h>
#include <stdint.h>
#include "types.h"

class webcontent {

private:

public:
    webcontent(void);
    char* replaceWord(const char* s, const char* oldW, const char* newW);
    void setSensorType(sensor_t sensorType);
    void setDNMSSensor(dnmssensor_t micType);
    void setDNMSMicOffset(float mic_offset);    
    void setSWVersion(String swVersion);
    void setLORAInfo(String APPEUI, String APPKEY, String DEVEUI);
    String getHomePage(void);
    String makePart_dnms(void);
    String makeSetupPage(void);
    String makeSetupReset(void);
};

