// GY-80 IMU Library
// Code written by Marcelo Maro�as @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 5, 2017
// Based on Jim Lindblom code.
// Contact : marcelomaronas at poli.ufrj.br
// For more codes : github.com/engmaronas

#include <Wire.h> //Comment this line if you already included in your code.
#include "Arduino.h" 

#ifndef GY80IMU
#define GY80IMU

typedef struct 
{
  double acelerometro[2]; //Posi��es 1,2,3, respectivamente s�o as Acelera��es em x,y,z
  int magnetometro[2]; //Posi��es 1,2,3, respectivamente s�o as Campos Magn�ticos em x,y,z
  int giroscopio[2]; //Posi��es 1, 2, 3, respectivamente s�o a velocidade angular em x,y,z
  double barometro[2]; //Posi��es 1,2,3 respectivamente s�o Pressao, Altura e Temperatura
}IMU_s; //IMU Structure

//BMP085 - Bar�metro

//Functions Header
void Get_IMU (IMU_s *pIMUs, bool DebugSerialMonitor);

void bmp085Calibration();

void Get_BMP (double *paltitude, double *ptemperatura, double *ppressao);

float bmp085GetTemperature(unsigned int ut);

long bmp085GetPressure(unsigned long up);

char bmp085Read(unsigned char address);

int bmp085ReadInt(unsigned char address);

unsigned int bmp085ReadUT();

unsigned long bmp085ReadUP();

void writeRegister(int deviceAddress, byte address, byte val);

int readRegister(int deviceAddress, byte address);

float calcAltitude(float pressure);

void Get_Accel(double *pAx, double *pAy, double *pAz );

void Get_Magneto(int *pMx, int *pMy, int *pMz);

int setupL3G4200D(int scale);

void Get_Gyro(int *pGx, int *pGy, int *pGz);

//Functions Header


#endif
