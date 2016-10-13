// Find and track a textured planar object.
#include "includes.h"
#include "FeatureFinder.h"
#include "Serial.h"

using namespace std;

int main(int argc, char* argv[]) {
	//////////////////////////////////////////
	//Configuration Values
	int vidInd = 0;
	string cameraFile = "robocam_calib.xml";
	string boardFile = "hybrid_circles_wide.xml";
	char* comport = "COM5";
	//////////////////////////////////////////

	//////////////////////////////////////////
	//Constants
	const int FIELD_LENGTH = 7380;
	const int FIELD_WIDTH = 3750;
	const int ROBOT_LENGTH = 750;
	const int ROBOT_WIDTH = 1250;
	const int AVERAGE_POINTS = 5;
	//////////////////////////////////////////
	
	
	cv::Mat videoFrame;

	cv::Mat grayscaleFrame;
	vector<cv::Point2f> features;

	cv::VideoCapture video(vidInd);
	if (!video.isOpened()){
		cout << "Could not access Camera\n";
		exit(1);
	}

	cv::Mat cameraMatrix, distCoeffs, map1, map2;
	cv::FileStorage fs(cameraFile, cv::FileStorage::READ);
	if (!fs.isOpened()){
		std::cout << "Could not read camera parameters\n";
		exit(1);
	}

	cv::Size size;
	//Loads camera paramters
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs["image_Height"] >> size.height;
	fs["image_Width"] >> size.width;
	fs.release();

	//Allows undistortion of camera
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, size, 1, size, 0),
		size, CV_16SC2, map1, map2);

	//Starts feature finder with desired toarget


	FeatureFinder Target(boardFile, cameraFile);
	// Checks if there should be a sub pattern file
	bool isMultiBoard = false;
	string altBoardFile;
	cv::FileStorage bf(boardFile, cv::FileStorage::READ);

	if (!bf.isOpened()){
		std::cout << "Could not read board parameters\n";
		exit(1);
	}
	bf["childBoard"] >> altBoardFile;

	FeatureFinder altTarget;
	if (altBoardFile.length() > 0) {
		isMultiBoard = true;
		altTarget = FeatureFinder(altBoardFile, cameraFile);
	}


	

	
	//Set camera position, change x, y, z. Based on center of camera
	Location center;
	cv::Mat rvec, tvec;
	deque<Location> locs;

	vector<cv::Point3f> targcenter;
	int axsize = 20;
	targcenter.push_back(cv::Point3f(0, 0, 0));
	targcenter.push_back(cv::Point3f(axsize, 0, 0));
	targcenter.push_back(cv::Point3f(0, axsize, 0));
	targcenter.push_back(cv::Point3f(0, 0, axsize));
	vector <cv::Point2f> piccenter;

	float theta = 0, phi = 0;
	Location robotLoc(0, 0, 0, 0, 0, 0); //Center of robot
	Location cameraLoc(200, 750, -150, 0, 0, 0);
	Location boardLoc;
	Location trueLoc;
	cameraLoc.setAngles(phi, theta, 0);

	//Creates serial communication with robot
	Serial superserial = Serial(comport);

	if (!superserial.IsConnected()) {
		cout << "Cannot access arduino, no tracking enabled" << endl;
	}
	else {
		//Starts camera at Theta = 0 Phi = 0
		char initdata[50];
		sprintf_s(initdata, "T%iP%i\n", (int)(theta * 1000), (int)(phi * 1000));
		cout << initdata << endl;
		superserial.WriteData(initdata, 50);
	}
	
	bool running = true;
	bool pause = false;
	bool lost = true;

	//Main loop
	while (running) {
		//Create illustration
		cv::Mat Field(FIELD_LENGTH / 10, FIELD_WIDTH / 10, CV_8UC3);
		Field.setTo(cv::Scalar(0, 0, 0));

		//Read video
		video.read(videoFrame); //Clears frame buffer, gets new frame
		video.read(videoFrame);

		cv::Mat vidframe2 = videoFrame.clone();
		cv::Mat debugFrame = videoFrame.clone();
		cv::Mat outputFrame = videoFrame.clone();
		cv::Rect PredictRect; //Unimplimented (for now)

		//Finds patterns
		center = Target.findPattern(vidframe2, PredictRect);
		debugFrame = Target.getDebugFrame();

		lost = true;
		if (Target.wasSuccessful()){
			lost = false;
		} else {
			if (isMultiBoard) {
				center = altTarget.findPattern(vidframe2, PredictRect);
				debugFrame = altTarget.getDebugFrame();
				if (altTarget.wasSuccessful()){
					lost = false;
				}
			}
		}

		if (!lost){ // z=0 means target inside camera, impossible
			// Finds target	
			cv::Mat boardTVec;
			cv::Mat boardRVec;
			cv::composeRT(center.getRvec(), center.getTvec(), cameraLoc.getRvec(), cameraLoc.getTvec(), boardRVec, boardTVec);
			boardLoc = Location(boardRVec, boardTVec);
			robotLoc = Location(boardLoc.getTF().inv());
			//cout << robotLoc.getLoc() << endl;

			//Adds location to saved location
			locs.push_back(robotLoc);

			int len = min(locs.size(), AVERAGE_POINTS);
			cv::Mat tempLocVec = robotLoc.getLoc().clone();
			
			for (int i = 1; i < len; i++){
				tempLocVec = tempLocVec + locs.at(locs.size() - 1 - i).getLoc();
			}
			trueLoc.setLoc(tempLocVec / len);

			//Outputs location to console (for debugging)
			//cout << robotLoc.getLoc() << endl;

			float centx = 0, centy = 0;

			

			cv::projectPoints(targcenter, center.getRvec(), center.getTvec(), cameraMatrix, distCoeffs, piccenter);

			centx = piccenter.at(0).x;
			centy = piccenter.at(0).y;

			cv::line(outputFrame, piccenter.at(0), piccenter.at(1), cv::Scalar(255, 0, 0), 2);
			cv::line(outputFrame, piccenter.at(0), piccenter.at(2), cv::Scalar(0, 255, 0), 2);
			cv::line(outputFrame, piccenter.at(0), piccenter.at(3), cv::Scalar(0, 0, 255), 2);
			cv::circle(outputFrame, piccenter.at(0), 5, cv::Scalar(0, 0, 0), 5);
			/*for (int i = 0; i < features.size(); i++){
				centx += features.at(i).x;
				centy += features.at(i).y;
			}
			centx *= 1 / ((double)features.size());
			centy *= 1 / ((double)features.size());*/

			//cout << "x: " << centx << "y: " << centy << endl;

			//cout << atan2((centx - cameraMatrix.at<double>(0, 2)), cameraMatrix.at<double>(0, 0)) << endl;

			if (superserial.IsConnected()){
				float dtheta = atan2((centx - cameraMatrix.at<double>(0, 2)), cameraMatrix.at<double>(0, 0));
				float dphi = atan2((centy - cameraMatrix.at<double>(1, 2)), cameraMatrix.at<double>(1, 1));

				//Calculates theta and phi
				/*float dist = sqrt(x*x + y*y + z*z);
				float theta = atan2((float)z, x);
				float phi = asin(((float)y) / dist);
				*/

				if (abs(dtheta) > PI / 64) {
					theta -= dtheta;
				}

				if (abs(dphi) > PI / 32) {
					phi -= dphi;
				}

				//Bounds theta and phi to -90 - 90
				if (theta < -PI) {
					theta = -PI;
				}
				else if (theta > PI){
					theta = PI;
				}

				if (phi < -PI / 2) {
					phi = -PI / 2;
				}
				else if (phi > PI / 2){
					phi = PI / 2;
				}

				//Writes sero data to arduino
				char data[50];
				sprintf_s(data, "T%iP%i\n", (int)(-theta * 1000), (int)(phi * 1000));
				bool result = superserial.WriteData(data, 50);
				//cout << data << endl;

				//Delays extra to give servo time to respond
				cvWaitKey(300);
			}			
		}		

		cv::Mat tipFrame;

		tipFrame = debugFrame * 0;

		for (int i = -1; i < 2; i++) {
			line(tipFrame, cv::Point(int(size.width / 2 - size.width / 3), size.height / 2 + i*size.height / 4), cv::Point(int(size.width / 2 + size.width / 3), size.height / 2 + i*size.height / 4), cv::Scalar(255, 255, 255));
		}


		double x, y, z, rot, yaw, roll;
		// Outputs path to screen

		if (locs.size() > 50) {
			locs.pop_front();
		}

		for (int i = 1; i < locs.size(); i++){
			cv::Point Point1 = cv::Point((locs.at(i).getX() + FIELD_WIDTH / 2) / 10, (locs.at(i).getZ()) / 10);
			cv::Point Point2 = cv::Point((locs.at(i - 1).getX() + FIELD_WIDTH / 2) / 10, (locs.at(i - 1).getZ()) / 10);
			line(Field, Point1, Point2, cv::Scalar(0, 0, 255));
			circle(Field, Point2, 2, cv::Scalar(255, 255, 255), 0);
		}

		line(Field, cv::Point(0, 1500 / 10), cv::Point(FIELD_WIDTH / 10, 1500 / 10), cv::Scalar(255, 255, 255));
		line(Field, cv::Point(0, 4440 / 10), cv::Point(FIELD_WIDTH / 10, 4440 / 10), cv::Scalar(255, 255, 255));
		cv::rectangle(Field, cv::Point(0, 0), cv::Point(FIELD_WIDTH / 10 - 1, FIELD_LENGTH / 10 - 1), cv::Scalar(255, 255, 255));

		// Outputs last location as robot
		if (locs.size() != 0) {
			int end = locs.size() - 1;	
			cv::Point Point2 = cv::Point((locs.at(end).getX() + FIELD_WIDTH / 2) / 10, (locs.at(end).getZ()) / 10);
			cv::Point Pointtrue = cv::Point((trueLoc.getX() + FIELD_WIDTH / 2) / 10, (trueLoc.getZ()) / 10);
			circle(Field, Point2, 2, cv::Scalar(255, 255, 255), 0);
			circle(Field, Pointtrue, 3, cv::Scalar(0, 255, 0), 3);

			cv::Mat angs = trueLoc.getAngles();
			double pitch = angs.at<double>(0);
			double yaw = angs.at<double>(1);
			double roll = angs.at<double>(2);
			// Draw Robot
			cv::RotatedRect Robotbox = cv::RotatedRect(Pointtrue, cv::Size(ROBOT_WIDTH/10, ROBOT_LENGTH/10), yaw);
			cv::Point2f vertices [4];
			Robotbox.points(vertices);
			for (int i = 0; i < 4; i++)
				cv::line(Field, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));

			// Draw pitch/roll
			cv::Point centerRoll = cv::Point(size.width / 2, size.height / 2 + pitch);

			cout << centerRoll << endl;
			cv::Point upRoll = cv::Point(-1.0 / 10.0 * size.height*sin(roll*PI/180), -1.0/ 10.0 * size.height*cos(roll*PI/180));
			cv::Point rightRoll = cv::Point(1.0 / 10.0 * size.width*cos(roll*PI/180), -1.0 / 10.0 * size.width*sin(roll*PI / 180));

			cout << upRoll << endl;
			cout << rightRoll << endl;

			cv::line(tipFrame, centerRoll, centerRoll + upRoll, cv::Scalar(0, 255, 0), 3);
			cv::line(tipFrame, centerRoll, centerRoll + rightRoll, cv::Scalar(0, 255, 0), 3);
			cv::line(tipFrame, centerRoll, centerRoll - rightRoll, cv::Scalar(0, 255, 0), 3);
		}
		cameraLoc.setAngles(phi, theta, 0);
		cv::Mat fieldSpacer;
		fieldSpacer = cv::Mat(videoFrame.size().height * 2 - FIELD_LENGTH / 10, FIELD_WIDTH / 10, CV_8UC3);


		//Build output
		cv::Mat panel, panel2;
		cv::vconcat(videoFrame, outputFrame, panel);
		cv::vconcat(debugFrame, tipFrame, panel2);
		cv::hconcat(panel, panel2, panel);
		cv::vconcat(Field, fieldSpacer*0, Field);
		cv::hconcat(panel, Field, panel);
		
		cv::imshow("Panel", panel);

		//Delays to show image
		int k = cv::waitKey(10);

		if (k == 'q') break;

	}

	return EXIT_SUCCESS;
}