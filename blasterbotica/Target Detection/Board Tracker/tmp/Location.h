#ifndef LOCATION_H
#define LOCATION_H
#include <opencv2\opencv.hpp>

const double PI = 3.1415926535;

class Location {
//Generalized location class, allows for quick generation of transformation matricies and transformation/rotation vectors based on coordinates
private:
	cv::Mat Xmat;

public:
	Location();
	Location(cv::Mat X);
	Location(cv::Mat rvec, cv::Mat tvec);
	Location(double xin, double yin, double zin, double axin, double ayin, double azin);
	cv::Mat getLoc();
	cv::Mat getRvec();
	cv::Mat getTvec();
	void setLoc(cv::Mat X);
	void setLoc(cv::Mat rvec, cv::Mat tvec);
	void setX(double x);
	void setY(double y);
	void setZ(double z);
	void setAX(double ax);
	void setAY(double ay);
	void setAZ(double az);
	double getX();
	double getY();
	double getZ();
	double getAX();
	double getAY();
	double getAZ();
	cv::Mat getR();
	cv::Mat getTF();
	cv::Mat Location::getAngles();
	void Location::setAngles(double ax, double ay, double az);

};

#endif