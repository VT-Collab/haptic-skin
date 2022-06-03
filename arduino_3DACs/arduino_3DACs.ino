#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Adafruit_MCP4725 dac3;

float P;
float V;
float temp_dac_input;
float dac_in_1;
float dac_in_2;
float dac_in_3;
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
      
      String p_1 = p_string.substring(0,1);
      String p_2 = p_string.substring(1,2);
      String p_3 = p_string.substring(2,3);

      dac_in_1 = getDAC_Input(p_1);
      dac_in_2 = getDAC_Input(p_2);
      dac_in_3 = getDAC_Input(p_3);
            
      dac1.setVoltage(dac_in_1, false);     
      dac2.setVoltage(dac_in_2, false);      
      dac3.setVoltage(dac_in_3, false);            
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



int getDAC_Input(String p){
  
      P = p.toFloat();
      Serial.println(P);       
      I = (P+7.5)*8.0/15.0;            
      V = (I-4.0)/3.2;              
      temp_dac_input = V*4095.0/5.0;        
      return (int)temp_dac_input;
}
