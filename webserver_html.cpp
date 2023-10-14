#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "types.h"
#include "hexutil.h"

#include "webserver_html.h"

static sensor_t mySensorType;
static dnmssensor_t myDNMSSensor;
static String mySWVersion;
static String myHtml_top;
static String myHtml_bottom;
static String myAPPEUI;
static String myAPPKEY;
static String myDEVEUI;
static float mymic_offset;

webcontent::webcontent(void)
{
    myHtml_top = R"(<!DOCTYPE HTML><html> <head><meta http-equiv=\"Refresh\"/></head>
<img src="../NoiceOrDust.png" alt='logo' width="400">
<h1>#sensorType#</h1>
<p>Digitale fijnstof of geluidsmeter gemaakt door:<br>
Bas van Drunen en Tonnie Derks
<p><p>Software version: #FIRMWARE_VERSION#
<p><p>LORA info:<br>
APPEUI: #APPEUI#<br>
APPKEY: #APPKEY#<br>
DEVEUI: #DEVEUI#<br>
<hr>
<br>)";
    myHtml_bottom = R"(<p>
<hr>
<p><button onclick="document.location.href='../update'">Update Software</button>
<p><button onclick="document.location.href='../setupreset'">Reset sensor</button>
</html>)";
}

void webcontent::setSWVersion(String swVersion)
{
    mySWVersion = swVersion;
}

void webcontent::setSensorType(sensor_t sensorType)
{
    mySensorType = sensorType;
}

void webcontent::setDNMSSensor(dnmssensor_t micType)
{
    myDNMSSensor = micType;
}

void webcontent::setDNMSMicOffset(float mic_offset)
{
    mymic_offset = mic_offset;
}

void webcontent::setLORAInfo(String APPEUI, String APPKEY, String DEVEUI)
{
    myAPPEUI = APPEUI;
    myAPPKEY = APPKEY;
    myDEVEUI = DEVEUI;
}

String webcontent::makePart_dnms(void)
{
    String index_html1 = R"(<p>Microphone: #microphone#<br>
<p>Offset correction: #micoffset# dB<br>
<p></p>
<p></p> <button onclick="document.location.href='../setup'">Change microphone</button>
<p></p><br><br>)";
    switch (myDNMSSensor) {
        case E_DNMS_NONE:
            index_html1 = replaceWord(index_html1.c_str(),"#microphone#","None, select mic by clicking on Change Setup");
            break;
        case E_DNMS_ICS43434:
            index_html1 = replaceWord(index_html1.c_str(),"#microphone#","ICS43434");
            char micoffset[8];
            sprintf(micoffset, "%.2f", mymic_offset);
            index_html1 = replaceWord(index_html1.c_str(),"#micoffset#",micoffset);
            break;
        case E_DNMS_SPH0645:
            index_html1 = replaceWord(index_html1.c_str(),"#microphone#","SPH0645");
            break;
    };
    return (index_html1);
}

String webcontent::getHomePage(void)
{
    myHtml_top = replaceWord(myHtml_top.c_str(), "#FIRMWARE_VERSION#",FIRMWARE_VERSION);

    myHtml_top = replaceWord(myHtml_top.c_str(), "#DEVEUI#",myDEVEUI.c_str());
    myHtml_top = replaceWord(myHtml_top.c_str(), "#APPEUI#",myAPPEUI.c_str());
    myHtml_top = replaceWord(myHtml_top.c_str(), "#APPKEY#",myAPPKEY.c_str());


    String html_middle;
    if (mySensorType == E_SENSOR_PMSENSOR) {
        myHtml_top = replaceWord(myHtml_top.c_str(), "#sensorType#","Particulare Matter Sensor");
        return (myHtml_top + myHtml_bottom);
    }
    if (mySensorType == E_SENSOR_DNMSSENSOR) {
        myHtml_top = replaceWord(myHtml_top.c_str(), "#sensorType#","Digital Noice Measurement Sensor");
        html_middle = makePart_dnms();
        return (myHtml_top + html_middle + myHtml_bottom);
    }
    return("");
}

String webcontent::makeSetupPage(void)
{
    String index_html = R"(
<p>Choose a microphone:</p>
<p>ICS43434</p> <button onclick="document.location.href='../save_ICS43434'">ICS43434</button>
<p>SPH0645</p> <button onclick="document.location.href='../save_SPH0645'">SPH0645</button>)";
    return (myHtml_top + index_html + myHtml_bottom);
}

String webcontent::makeSetupReset(void)
{
    String index_html = R"(
Clicking the button below will reset to sensor type <b>unknown</b></p>
Hardware will switch to PM when PM sensor found, otherwise switch to DNMS.<br>
Calibration for the microphone offset is necessary after a reset!.
<p>
<button onclick="document.location.href='../reset'" ; style="background-color:red">Reset</button>)";
    return (myHtml_top + index_html + "</html>");
}

// Function to replace a string with another 
// string 
char* webcontent::replaceWord(const char* s, const char* oldW, 
                const char* newW) 
{ 
    char* result; 
    int i, cnt = 0; 
    int newWlen = strlen(newW); 
    int oldWlen = strlen(oldW); 
  
    // Counting the number of times old word 
    // occur in the string 
    for (i = 0; s[i] != '\0'; i++) { 
        if (strstr(&s[i], oldW) == &s[i]) { 
            cnt++; 
  
            // Jumping to index after the old word. 
            i += oldWlen - 1; 
        } 
    } 
  
    // Making new string of enough length 
    result = (char*)malloc(i + cnt * (newWlen - oldWlen) + 1); 
  
    i = 0; 
    while (*s) { 
        // compare the substring with the result 
        if (strstr(s, oldW) == s) { 
            strcpy(&result[i], newW); 
            i += newWlen; 
            s += oldWlen; 
        } 
        else
            result[i++] = *s++; 
    } 
  
    result[i] = '\0'; 
    return result; 
}

