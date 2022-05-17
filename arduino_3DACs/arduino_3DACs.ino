#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Adafruit_MCP4725 dac3;

float P;
float V;
float temp_dac_input;
float dac_in;
float I;


const byte numChars = 6;
char receivedChars[numChars];
int chr_size;
char ex_char = 'a';
String p_string;
boolean newData = false;

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(2);
    dac1.begin(0x60);
    dac2.begin(0x62);
    dac3.begin(0x63);    
}


void loop() {
  if (Serial.available())
  { 
    recvWithStartEndMarkers();
    p_string = showNewData();
    if (p_string != "")
    {
      P = p_string.toFloat();
      Serial.println(P);      
      I = (P+7.5)*8.0/15.0;      
      V = (I-4.0)/3.2;    
          
      temp_dac_input = V*4095.0/5.0;        
      dac_in = (int)temp_dac_input;
            
      dac1.setVoltage(dac_in, false);
      dac2.setVoltage(dac_in, false);
      dac3.setVoltage(dac_in, false);
            
    } 
  }
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


String showNewData() {
    String s;
    if (newData == true) {
//        Serial.print("This just in ... ");
//        Serial.println(receivedChars);
        newData = false;

        int ch_size;
        char temp = 'a';
        
        ch_size = sizeof(receivedChars)/sizeof(temp);
        s = convertToString(receivedChars, ch_size);        
    }
    return s;
}


String convertToString(char* a, int size)
{
    int i;
    String s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}
