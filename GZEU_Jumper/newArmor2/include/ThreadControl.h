#pragma once
#include <thread>
#include <atomic>
//#include <X11/Xlib.h>
#include "../include/serial.hpp"
#include "../include/Armor.h"
#include <opencv2/opencv.hpp>
#include <time.h>
//#include "Predict.h"
class processer{
public:
	processer(){};
	~processer(){};

	void getUartData();
	
	void frameLoop();




};
