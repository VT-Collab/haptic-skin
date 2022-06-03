// Norawish Lohitnavy & Antonio Alvarez Valdivia

// This program controls the voltage output of a PWM pin to feed an OpAmp that
// generates a control voltage signal for 550x pressure regulator.
// The program also reads the analog signal of a HSCDANN030PGAA5 Honeywell 
// pressure sensor.  

// The user input a voltage value in the serial monitor, and the required PWM
// is generated in the output. The serial monitor then shows pressure readings 
// at 9600 baud rate.

int signal1 = 3;
int p_check = A1;

double temp = 0;
double prev_des_pressure = 0;

float desired_voltage = 0;  //V
double value = 0;
int AnalogOut = 0;

#define MAX_VOLTAGE 5.0
#define ANALOG_WRITE_COUNTS 255

void setup() {

  Serial.begin(9600);
  
}

void loop() {
  AnalogOut = analogRead(p_check);
  Serial.println( ((AnalogOut * (5.0/1023.0)) - 0.5)* (30.0/4.0)); //for 30psi sensor
//  Serial.println( ((AnalogOut * (5.0/1023.0)) - 0.5)* (5.0/4.0)); //for 5psi sensor

}

  
