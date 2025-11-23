#pragma once
#include "lib\BMP3_SensorAPI-master\BMP3_SensorAPI-master\bmp3.h"


/**
*@brief initBMP390 initialises the BMP390 temperature and pressure sensor
*with the FIFO data structure. 
*@return 1 if successful, 0 if error. 
*
*/
int initBMP390();