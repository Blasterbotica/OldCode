#include "Location.h"


Location::Location() {
	//Creates a blank location vector
	this->Xmat = (cv::Mat_<double>(6, 1) << 0, 0, 0, 0, 0, 0);
}

Location::Location(cv::Mat X) {
	//Loads a location from either a location vector or a transformation matrix
	Xmat = (cv::Mat_<double>(6, 1) << 0, 0, 0, 0, 0, 0);

	if (X.rows == 6 && X.cols == 1){
		//Takes a location vector and loads it
		this->Xmat = X;
	}
	else if (X.rows == 4 && X.cols == 4) {
		//Takes a transformation matrix and loads it as a location vector

		Xmat.at<double>(0,0) = X.at<double>(0, 3);	//tx
		Xmat.at<double>(1,0) = X.at<double>(1, 3);	//ty
		Xmat.at<double>(2,0) = X.at<double>(2, 3);	//tz

		cv::Mat rvec;
		cv::Rodrigues(X(cv::Rect(0, 0, 3, 3)), rvec);
		Xmat.at<double>(3, 0) = rvec.at<double>(0);
		Xmat.at<double>(4, 0) = rvec.at<double>(1);
		Xmat.at<double>(5, 0) = rvec.at<double>(2);
	}
		
}

Location::Location(cv::Mat rvec, cv::Mat tvec) {
	//Creates a location vector from a rotation vector and a translation vector
	cv::vconcat(tvec, rvec, this->Xmat);
}

Location::Location(double x, double y, double z, double ax, double ay, double az){
	// Creates a location object from blank inputs
	this->Xmat = (cv::Mat_<double>(6, 1) << x, y, z, ax, ay, az);
}

cv::Mat Location::getLoc(){
	// Returns the location vector
	return this->Xmat;
}

cv::Mat Location::getRvec(){
	// Returns the rotation vector
	return Xmat(cv::Rect(0, 3, 1, 3));
}

cv::Mat Location::getTvec(){
	// Returns the translation vector
	return Xmat(cv::Rect(0, 0, 1, 3));
}

cv::Mat Location::getTF(){
	// Creates and returns a transformation matrix (4x4)
	cv::Mat R, Tf;
	cv::Rodrigues(this->getRvec(), R);

	hconcat(R, this->getTvec(), Tf);
	Tf.resize(4);
	Tf.at<double>(3, 0) = 0;
	Tf.at<double>(3, 1) = 0;
	Tf.at<double>(3, 2) = 0;
	Tf.at<double>(3, 3) = 1;
	return Tf;
}

void Location::setLoc(cv::Mat X){
	// Sets a new location based on a location vector
	this->Xmat = X;
}

void Location::setLoc(cv::Mat rvec, cv::Mat tvec){
	// Sets a new location based on a rotation and translation vector
	vconcat(tvec, rvec, this->Xmat);
}

// Sets rotation vector based on provided angles (different from rotation vector components)
void Location::setAngles(double ax, double ay, double az){
	cv::Mat Rx, Ry, Rz, R, rvec;

	Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(ax), -sin(ax), 0, sin(ax), cos(ax));
	Ry = (cv::Mat_<double>(3, 3) << cos(ay), 0, sin(ay), 0, 1, 0, -sin(ay), 0, cos(ay));
	Rz = (cv::Mat_<double>(3, 3) << cos(az), -sin(az), 0, sin(az), cos(az), 0, 0, 0, 1);
	R = Rx*Ry*Rz;
	cv::Rodrigues(R, rvec);
	this->setLoc(rvec, this->getTvec());

}

// Individual getters and setters for location vector components
void Location::setX(double x){
	this->Xmat.at<double>(0,0) = x;
}

void Location::setY(double y){
	this->Xmat.at<double>(1, 0) = y;
}

void Location::setZ(double z){
	this->Xmat.at<double>(2, 0) = z;
}

void Location::setAX(double ax){
	this->Xmat.at<double>(3, 0) = ax;
}

void Location::setAY(double ay){
	this->Xmat.at<double>(4, 0) = ay;
}

void Location::setAZ(double az){
	this->Xmat.at<double>(5, 0) = az;
}

double Location::getX(){
	return this->Xmat.at<double>(0, 0);
}

double Location::getY(){
	return this->Xmat.at<double>(1, 0);
}

double Location::getZ(){
	return this->Xmat.at<double>(2, 0);
}

double Location::getAX(){
	return this->Xmat.at<double>(3, 0);
}

double Location::getAY(){
	return this->Xmat.at<double>(4, 0);
}

double Location::getAZ(){
	return this->Xmat.at<double>(5, 0);
}

