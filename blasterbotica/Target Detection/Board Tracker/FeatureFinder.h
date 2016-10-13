#include "includes.h"

class FeatureFinder{
private:
	int shapeType;
	vector<cv::Point3f> altShape;
	std::map<int, vector<cv::Point3f>> targetDB;
	vector<cv::Vec4i> charucoIds;
	vector<cv::Point3f> targetshape;
	cv::Mat lastFrame;
	cv::Mat debugFrame;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat map1;
	cv::Mat map2;
	bool success;
	double scale;
	int nlocs;
	int lostcount;
	cv::KalmanFilter KF;


public:
	FeatureFinder();
	FeatureFinder(string boardFile, string cameraFile);
	Location findPattern(cv::Mat videoFrame, cv::Rect PredictRect);
	vector<cv::Point2f> findCircles(cv::Mat videoFrame);
	vector<cv::Point2f> findAruco(cv::Mat videoFrame);
	vector<cv::Point2f> findChAruco(cv::Mat videoFrame);
	cv::Mat getLastFrame();
	cv::Mat getDebugFrame();
	bool wasSuccessful();
};
