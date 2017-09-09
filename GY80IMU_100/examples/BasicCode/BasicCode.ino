/*  Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 6, 2017
 *  This is a basic example that uses the function Get_IMU to store the GY-80 variables in a structure.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[2]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[2]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[2]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[2]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.    
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas
 */

/* GY-80 Pins
 *  Vcc_In <----------------------> Arduino 5v
 *  Gnd    <----------------------> Arduino Gnd
 *  SDA    <----------------------> A4
 *  SCL    <----------------------> A5
 */

#include <GY80IMU.h> //Include the library GY80IMU

//Use this variables to enable debugging via Serial Monitor
bool DebugSerial = 0;  //Prints the GY80 values from inside the "Get_IMU()" function
bool DebugSDSerial = 1; //Prints the values stored in the structure IMU_s

//Structure declaration
IMU_s IMUs1; 
IMU_s *pIMUs1 = &IMUs1;

//Modify the Delay_Time variable to control how much info is printed on the serial monitor
float Delay_Time = 500;

void setup() {
  Serial.begin(115200); //Starts the serial port at 115200 baud rate
  Wire.begin();

  //
  Serial.println("GY-80 IMU library ");
  Serial.println("by Marcelo Maronas");
  Serial.println();
  //

  Serial.println("sep =, "); //This line handles Excel CSV configuration.
  Serial.println("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"); 
}

void loop() {
  //Stores the GY80 values into the pIMUs1 pointer
   Get_IMU(pIMUs1, DebugSerial);

  //Delay time
   delay(Delay_Time);
   
  //if DebugSDSerial = 1, write into the Serial Monitor
  if (DebugSDSerial) {
    Serial.print(millis());Serial.print(" ,");
    Serial.print(pIMUs1->barometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->barometro[1]);Serial.print(" ,");
    Serial.print(pIMUs1->barometro[2]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[1]);Serial.print(" ,");
    Serial.print(pIMUs1->acelerometro[2]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[0]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[1]);Serial.print(" ,");
    Serial.print(pIMUs1->giroscopio[2]);Serial.print(" ,");
    Serial.print(pIMUs1->magnetometro[0]);Serial.print(" ,");
    Serial.print(pIMUs1->magnetometro[1]);Serial.print(" ,");
    Serial.println(pIMUs1->magnetometro[2]);
}
}
