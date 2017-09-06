// GY-80 IMU Library
// Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 5, 2017
// Based on Jim Lindblom code.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include "GY80IMU.h"

//*****************L3G4200D Variables******************
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

//Endereco I2C do L3G4200D
int L3G4200D_Address = 105;

//*****************End of L3G4200D Variables****************

//*****************HMC5883L Variables******************
#define HMCaddress 0x1E
//*****************End of HMC5883L Variables****************

//*****************ADXL345 Variables******************
#define Register_ID 0
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37

// Endereco I2C do sensor : 83 em decimal ou 0x53
int ADXAddress = 0x53;  // the default 7-bit slave address
//*****************End of ADXL345 Variables******************

//***************BMP085 Variables*******************
//BMP085_ADRESS = "0x77" - I2C address for BMP085

const unsigned char OSS = 0; // Oversampling Setting 

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this 
// variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;
//***************End of BMP085 Variables*******************

//****************BMP085 Code************************
void Get_BMP (double *paltitude, double *ptemperatura, double *ppressao)
{       
        // Inicializa o BMP085
        bmp085Calibration();
        float altitude;
        double temperatura;
	// Chama a rotina que calcula a temperatura
	// Esta rotina DEVE ser executada primeiro
	temperatura = double(bmp085GetTemperature(bmp085ReadUT()));
	// Chama a rotina que calcula a pressao
	float pressure = bmp085GetPressure(bmp085ReadUP());
	// Chama a rotina que calcula atmosfera
	float atm = pressure / 101325; 
	// Chama a rotina que calcula a altitude
	altitude = calcAltitude(pressure); 
	*ppressao = double(pressure);
	*paltitude = altitude;
	*ptemperatura = temperatura;
}

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calcula a temperatura em graus C
float bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calcula B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calcula B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(0x77);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(0x77);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

float calcAltitude(float pressure)
{
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
//**************************End of BMP085 Code****************

//**************************Gyroscope Code********************
void Get_Gyro(int *pGx, int *pGy, int *pGz)
{
  int Gx, Gy, Gz;
  setupL3G4200D(2000); // Fazer função de setup
  // Rotina para leitura dos valores de X, Y e Z
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  Gx = ((xMSB << 8) | xLSB);
  *pGx = Gx;

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  Gy = ((yMSB << 8) | yLSB);
  *pGy = Gy;

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  Gz = ((zMSB << 8) | zLSB);
  *pGz = Gz;

  //Serial.print("Giroscopio :");
  //Serial.print("x: ");
  //Serial.print(Gx);
  //Serial.print("  y: ");
  //Serial.print(Gy);
  //Serial.print("  z: ");
  //Serial.println(Gz);
}

int setupL3G4200D(int scale)
{
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}
//**************************End of Gyroscope Code**************

//**************************Accelerometer Code****************
void Get_Accel(double *pAx, double *pAy, double *pAz ){
  //Local variables
  int reading = 0;
  int val=0;
  int X0,X1,X_out;
  int Y0,Y1,Y_out;
  int Z1,Z0,Z_out;
  double Xg,Yg,Zg;
  
  Wire.beginTransmission(ADXAddress);
  Wire.write(Register_2D);
  Wire.write(8);                //measuring enable
  Wire.endTransmission();     // stop transmitting
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_X0);
  Wire.write(Register_X1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    X0 = Wire.read();
    X1 = Wire.read(); 
    X1=X1<<8;
    X_out=X0+X1;   
  }

  //------------------Y
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Y0);
  Wire.write(Register_Y1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Y0 = Wire.read();
    Y1 = Wire.read(); 
    Y1=Y1<<8;
    Y_out=Y0+Y1;
  }
  //------------------Z
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Z0);
  Wire.write(Register_Z1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Z0 = Wire.read();
    Z1 = Wire.read(); 
    Z1=Z1<<8;
    Z_out=Z0+Z1;
  }
  //
  Xg=(X_out/256.0)*9.30157; //Const 9.30157 is the calibration value for 9.8066 m/s² gravity
  Yg=(Y_out/256.0)*9.30157;
  Zg=(Z_out/256.0)*9.30157;
  *pAx = Xg;
  *pAy = Yg;
  *pAz = Zg;
  //Serial.print("Acelerometro em m/s2 : ");
  //Serial.print("X= ");
  //Serial.print(Xg);
  //Serial.print("       ");
  //Serial.print("Y= ");
  //Serial.print(Yg);
  //Serial.print("       ");
  //Serial.print("Z= ");
  //Serial.print(Zg);
  //Serial.println("  ");
}
//************************End of Accelerometer Code*****************************

//************************HMC5883L Code*************************************
void Get_Magneto(int *pMx, int *pMy, int *pMz) {
  Wire.begin();
  
  // Inicializa o HMC5883
  Wire.beginTransmission(HMCaddress);
  // Seleciona o modo
  Wire.write(0x02); 
  // Modo de medicao continuo
  Wire.write(0x00); 
  Wire.endTransmission();
  int x,y,z; //triple axis data
  
  // Indica ao HMC5883 para iniciar a leitura
  Wire.beginTransmission(HMCaddress);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 
  // Le os dados de cada eixo, 2 registradores por eixo
  Wire.requestFrom(HMCaddress, 6);
  if(6<=Wire.available())
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  *pMx = x;
  *pMy = y;
  *pMz = z;
  // Imprime os vaores no serial monitor
  //Serial.print("Magnetometro : ");
  //Serial.print("x: ");
  //Serial.print(x);
  //Serial.print("  y: ");
  //Serial.print(y);
  //Serial.print("  z: ");
  //Serial.println(z);
  
}
//************************End of HMC5883L Code*************************************

//************************IMU Structure Code***************************************
void Get_IMU (IMU_s *pIMUs, bool DebugSerialMonitor) //Preenche uma structure de IMU
{
  //Variáveis do BMP
  double pressao, altitudef, temperatura;
  double *ppressao = &pressao;
  double *paltitudef = &altitudef;
  double *ptemperatura = &temperatura;

  //Variáveis de Gyro
  int Gx, Gy, Gz;
  int *pGx = &Gx;
  int *pGy = &Gy;
  int *pGz = &Gz;


  //Variáveis de Accel
  double Ax, Ay, Az;
  double *pAx = &Ax;
  double *pAy = &Ay;
  double *pAz = &Az;

  //Vairáveis de Magneto
  int Mx, My, Mz;
  int *pMx = &Mx;
  int *pMy = &My;
  int *pMz = &Mz;

  Get_BMP(paltitudef, ptemperatura, ppressao);
  pIMUs->barometro[0] = pressao;
  pIMUs->barometro[1] = altitudef;
  pIMUs->barometro[2] = temperatura;

  Get_Magneto(pMx, pMy, pMz);
  pIMUs->magnetometro[0] = Mx;
  pIMUs->magnetometro[1] = My;
  pIMUs->magnetometro[2] = Mz;

  Get_Accel(pAx, pAy, pAz);
  pIMUs->acelerometro[0] = Ax;
  pIMUs->acelerometro[1] = Ay;
  pIMUs->acelerometro[2] = Az;
  
  Get_Gyro(pGx, pGy, pGz);
  pIMUs->giroscopio[0] = Gx;
  pIMUs->giroscopio[1] = Gy;
  pIMUs->giroscopio[2] = Gz;

if (DebugSerialMonitor) 
{
  Serial.print("Magnetometro : ");
  Serial.print("x : ");
  Serial.print(pIMUs->magnetometro[0]);
  Serial.print("  y : ");
  Serial.print(pIMUs->magnetometro[1]);
  Serial.print("  z : ");
  Serial.println(pIMUs->magnetometro[2]);

  Serial.print("Acelerometro em m/s2 : ");
  Serial.print("x : ");
  Serial.print(pIMUs->acelerometro[0]);
  Serial.print("  y : ");
  Serial.print(pIMUs->acelerometro[1]);
  Serial.print("  z : ");
  Serial.println(pIMUs->acelerometro[2]);
  
  Serial.print("Giroscopio em m/s : ");
  Serial.print("x : ");
  Serial.print(pIMUs->giroscopio[0]);
  Serial.print("  y : ");
  Serial.print(pIMUs->giroscopio[1]);
  Serial.print("  z : ");
  Serial.println(pIMUs->giroscopio[2]);

  Serial.print("BMP085 : ");
  Serial.print("Pressao em Pa : ");
  Serial.print(pIMUs->barometro[0]);
  Serial.print("  Altitude em Metros: ");
  Serial.print(pIMUs->barometro[1]);
  Serial.print("  Temperatura em Celsius: ");
  Serial.println(pIMUs->barometro[2]);
  Serial.println();
}

}