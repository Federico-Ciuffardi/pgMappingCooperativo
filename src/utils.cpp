#include <sstream>
#include <string>
#include <list>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
//#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


static float distacia2Puntos(cv::Point2f a, cv::Point2f b){
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	return sqrt(pow(dx,2) + pow(dy,2));
}

static int signo(int i){
	int ret = 0;
	if (i < 0){
		ret = -1;
	}else if (i > 0){
		ret = 1;
	}
	return ret;
}

/*Devuelve una posicion relativa a mi celda
i-1+width		i+width		i+1+width
	i-1					i					i+1
i-1-width 	i-width		i+1-width
*/
static int posicionRelativa(int indice, int pos, uint ancho){
	int ret = indice;
	switch ( pos ) {
		case 0:
			ret = indice-1-ancho;
			break;
		case 1:
			ret = indice-ancho;
			break;
		case 2:
			ret = indice+1-ancho;
			break;
		case 3:
			ret = indice-1;
			break;
		case 5:
			ret = indice+1;
			break;
		case 6:
			ret = indice-1+ancho;
			break;
		case 7:
			ret = indice+ancho;
			break;
		case 8:
			ret = indice+1+ancho;
			break;
	}
	return ret;
}
