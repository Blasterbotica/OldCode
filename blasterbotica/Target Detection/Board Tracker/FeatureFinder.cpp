#include "FeatureFinder.h"

bool isEllipse(vector<cv::Point> contour){
	//FIXME//
	cv::RotatedRect el;
	double areaIdeal, areaActual, perimeterActual;
	cv::Size dias;
	bool rval = false;
	areaActual = contourArea(contour);
	perimeterActual = cv::arcLength(contour, true);
	if (areaActual > 4) {
		//Check if ellipse
		if ((4 * 3.14159 *areaActual / (perimeterActual*perimeterActual) > 0.55)){
			el = fitEllipse(contour);
			dias = el.size;
			areaIdeal = dias.area() * 3.14159 / 4;
			if (abs(areaIdeal - areaActual) / areaIdeal > 0.10) {
				rval = false;
			}
			else {
				rval = true;
			}
		}

	}

	return rval;
}

double mag(cv::Point2d point){
	return sqrt(pow(point.x, 2) + pow(point.y, 2));
}

FeatureFinder::FeatureFinder(){
	shapeType = -1;
	nlocs = 0;
	targetshape.clear();
	targetDB.clear();
}
FeatureFinder::FeatureFinder(string boardFile, string cameraFile){
	shapeType = -1;
	nlocs = 0;
	targetshape.clear();
	targetDB.clear();
	
	cv::FileStorage bf(boardFile, cv::FileStorage::READ);

	if (!bf.isOpened()){
		cout << "Could not read target shape\n";
	}
	else {
		cv::Size targetsize;
		cv::Mat ids, locations, tempAlt;

		bf["nlocs"] >> nlocs;
		bf["type"] >> shapeType;
		bf["ids"] >> ids;
		bf["locations"] >> locations;


		bf.release();

		for (int i = 0; i < nlocs; i++){
			cv::Mat tempmat;
			vector<cv::Point3f> tempshape;
			tempmat = locations.row(i);
			for (int j = 0; j < locations.size().width; j++) tempshape.push_back(tempmat.at<cv::Point3f>(j));
			if (shapeType == 2) {
				cv::Vec4i tempvec = cv::Vec4i(ids.row(i));
				charucoIds.push_back(tempvec);
				targetDB[i] = tempshape;
			}
			else {
				targetDB[ids.at<int>(i)] = tempshape;
			}


		}
	}

	cv::FileStorage cf(cameraFile, cv::FileStorage::READ);
	cv::Size size;
	if (!cf.isOpened()){
		cout << "Could not read target shape\n";
	}
	else {
		cf["Camera_Matrix"] >> cameraMatrix;
		cf["Distortion_Coefficients"] >> distCoeffs;
		cf["image_Height"] >> size.height;
		cf["image_Width"] >> size.width;
		cf.release();
	}

	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, size, 1, size, 0),
		size, CV_16SC2, map1, map2);

}

Location FeatureFinder::findPattern(cv::Mat videoFrame, cv::Rect PredictRect) {
	Location output;
	vector<cv::Point2f> cameraPoints;
	cv::Mat fixedFrame;
	bool found = false;
	lastFrame = videoFrame;
	success = false;

	if (shapeType == 0){
		if (nlocs == 5){
			vector<cv::Point2f> circles;
			circles = findCircles(videoFrame);
			cameraPoints = circles;
			lastFrame = videoFrame;
			//cv::imshow("output", videoFrame);
			if (circles.size() == nlocs) {
				found = true;
			}

		}
		else {
			cout << "Improper number of markers for circle target\n";
		}
		
	}
	else if (shapeType == 1){
		cameraPoints = findAruco(videoFrame);
		if ((cameraPoints.size() > 0) && (cameraPoints.size() % 4) == 0){
			found = true;
		}
	}
	else if (shapeType == 2){
		cameraPoints = findChAruco(videoFrame);
		if ((cameraPoints.size() > 0) && (cameraPoints.size() % 4) == 0){
			found = true;
		}
		
	}
	else {
		cout << "Undefined shape type" << endl;

	}

	if (found){
		cv::Mat rvec, tvec;
		bool result = solvePnP(targetshape, cameraPoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
		if (result) {
			success = true;
			output = Location(rvec, tvec);
		}
	}

	return output;
}

bool FeatureFinder::wasSuccessful(){
	return success;
}

cv::Mat FeatureFinder::getDebugFrame(){
	return debugFrame;
}

cv::Mat FeatureFinder::getLastFrame(){
	return lastFrame;
}

vector<cv::Point2f> FeatureFinder::findCircles(cv::Mat videoFrame){
	//Circle finding algorithms adapted from Dr. Hoff
	cv::Mat grayFrame, imageThresh, ColorGrayFrame;
	cvtColor(videoFrame, grayFrame, CV_RGB2GRAY);
	equalizeHist(grayFrame, grayFrame);
	cvtColor(grayFrame, ColorGrayFrame, CV_GRAY2RGB);

	adaptiveThreshold(grayFrame,
		 imageThresh, // output thresholded image
		 255, // output value where condition met
		 cv::ADAPTIVE_THRESH_GAUSSIAN_C, // local neighborhood
		 cv::THRESH_BINARY_INV, // threshold_type - invert
		 91, // blockSize (any large number)
		 0); // a constant to subtract from mean

	// Apply morphological operations to get rid of small (noise) regions
	cv::Mat structuringElmt = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
	cv::Mat imageOpen;
	morphologyEx(imageThresh, imageOpen, cv::MORPH_OPEN, structuringElmt);
	cv::Mat imageClose;
	morphologyEx(imageOpen, imageClose, cv::MORPH_CLOSE, structuringElmt);
	// Now find connected components
	//imshow("whatIsee", imageClose);
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(imageClose, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	vector<cv::RotatedRect> circles;
	cv::RotatedRect el1, el2;
	double dist;
	cv::Point2d diff;
	cv::Size rads;
	vector<cv::Point2f> output;
	for (int i=0; i < contours.size(); i++){
		//Check for inner circle
		int outer = hierarchy.at(i)(3);
		if (outer == -1) continue;

		//Check if ellipse
		if (!isEllipse(contours.at(i))) continue;
		if (!isEllipse(contours.at(outer))) continue;
		

		//Get ellipses
		el1 = fitEllipse(contours.at(i));
		el2 = fitEllipse(contours.at(outer));

		//Check Centers are the same
		diff = el1.center - el2.center;
		dist = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
		if (dist > 5) continue;

		//Check ellipticalness match
		cv::Size s1 = el1.size;
		cv::Size s2 = el2.size;
		if (abs(s1.width / (1.0*s2.height) - s1.width / (1.0*s2.height)) / (s1.width / (1.0*s2.height)) > 0.1) continue;

		//Check areas
		double rat = s2.area()/s1.area();
		if (rat < 1.2 || rat > 4.5) continue;

		//SUCCESS
		circles.push_back(el2);	
		ellipse(ColorGrayFrame, el1, cv::Scalar(255, 0, 0),3);
		ellipse(ColorGrayFrame, el2, cv::Scalar(0, 0, 255),3);
		circle(ColorGrayFrame, el1.center, 1, cv::Scalar(0,255,0),0);
	}
	
	//imshow("grayframe", ColorGrayFrame);
	bool solved = false;
	if (circles.size() == 5) {
		putText(ColorGrayFrame, "Determining...", cv::Point(10,10), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 3);
		vector<int> posns;
		for (int j = 0; j < 5; j++){
			if (solved) break;
			for (int k = 0; k < 5; k++){
				if (solved) break;
				if (j == k) continue;
				for (int l = 0; l < 5; l++){
					if (solved) break;
					if (l == j || l == k) continue;
					//Determines if j k l are top thee circles (collinera, with l in the center
					cv::Point2d jloc = circles.at(j).center;
					cv::Point2d kloc = circles.at(k).center;
					cv::Point2d lloc = circles.at(l).center;
					cv::Point2d top = kloc - jloc;
					cv::Point2d halftop = lloc - jloc;

					if (abs(top.cross(halftop)/(mag(top)*mag(halftop))) > 0.05) continue;
					if (mag(top) < mag(halftop)) continue;

					for (int m = 0; m < 5; m++){

						if (m == l || m == k || m == j) continue;
						
						cv::Point2d mloc = circles.at(m).center;
						cv::Point2d dist1 = mloc - jloc;
						cv::Point2d dist2 = kloc - jloc;
						cv::Point2d dist3 = mloc - kloc;
						//Determines that m is closer to j than k
						if (mag(dist3) < mag(dist1)) continue;

						//Determines if circle is above or below line
						double area = dist1.cross(dist2);

						// Assigns positions
						if (area < 0){
							posns.push_back(j);
							posns.push_back(l);
							posns.push_back(k);
							posns.push_back(m);
							posns.push_back(10 - j - l - k - m);
						}
						else {
							posns.push_back(k);
							posns.push_back(l);
							posns.push_back(j);
							posns.push_back(10 - j - l - k - m);
							posns.push_back(m);
						}
						solved = true;
						break;

					}

				}
			}
		}
		if (solved) {
			char ptinfo[10];
			targetshape.clear();
			for (int j = 0; j < 5; j++){
				//Draws circles and arranges them in correct order
				int k = posns.at(j);
				targetshape.insert(targetshape.end(), targetDB[j].begin(), targetDB[j].end());
				output.push_back(circles.at(k).center);
				sprintf(ptinfo, "pt. %i", j);
				putText(ColorGrayFrame, ptinfo, circles.at(k).center, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 3);
			}
		}
	}
	else {
	}
	//Shows serach results
	debugFrame = ColorGrayFrame;
	return output;
}

vector<cv::Point2f> FeatureFinder::findAruco(cv::Mat videoFrame){
	cv::Mat grayFrame, bw, ColorGrayFrame;
	cvtColor(videoFrame, grayFrame, CV_RGB2GRAY);
	equalizeHist(grayFrame, grayFrame);
	cvtColor(grayFrame, ColorGrayFrame, CV_GRAY2RGB);
	targetshape.clear();
	vector<int>markerIds; 
	vector<cv::Point2f> output;
	vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::aruco::DetectorParameters parameters;
	parameters.adaptiveThreshConstant = 1;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	//Serches for 5x5 markers 0-49 with default serarch parameters
	cv::aruco::detectMarkers(grayFrame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

	if (markerIds.size() > 0){
		//Draws detected markers
		cv::aruco::drawDetectedMarkers(ColorGrayFrame, markerCorners, markerIds);
		for (int i = 0; i < markerIds.size(); i++){
			std::map<int, vector<cv::Point3f>>::iterator it;
			int j = markerIds.at(i);
			it = targetDB.find(j);
			if (it == targetDB.end()){
				continue;
			}
			targetshape.insert(targetshape.end(), targetDB[j].begin(), targetDB[j].end());
			output.insert(output.end(), markerCorners.at(i).begin(), markerCorners.at(i).end());
		}	
	}
	debugFrame = ColorGrayFrame;
	return output;

}

vector<cv::Point2f> FeatureFinder::findChAruco(cv::Mat videoFrame){
	cv::Mat grayFrame, ColorGrayFrame;
	cvtColor(videoFrame, grayFrame, CV_RGB2GRAY);
	equalizeHist(grayFrame, grayFrame);
	cvtColor(grayFrame, ColorGrayFrame, CV_GRAY2RGB);
	vector<int> markerIds;
	vector<cv::Point2f> output;
	targetshape.clear();
	vector<vector<cv::Point2f>> markerCorners, diamondCorners;
	vector<cv::Vec4i> diamondIds;
	cv::Mat rvec, tvec;
	cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	cv::aruco::detectMarkers(videoFrame, dict, markerCorners, markerIds);

	if (markerIds.size() > 0){
		
		cout << "Charucocheck\r\n";
		cv::aruco::detectCharucoDiamond(videoFrame, markerCorners, markerIds, 1.5, diamondCorners, diamondIds);
		cv::aruco::drawDetectedCornersCharuco(ColorGrayFrame, diamondCorners, diamondIds);
		for (int i = 0; i < diamondCorners.size(); i++){
			std::map<cv::Vec4i, int >::iterator it;
			int j = find(charucoIds.begin(), charucoIds.end(), diamondIds.at(i)) - charucoIds.begin();
			if (j >= charucoIds.size()) {
				continue;
			}
			targetshape.insert(targetshape.end(), targetDB[j].begin(), targetDB[j].end());
			output.insert(output.end(), diamondCorners.at(i).begin(), diamondCorners.at(i).end());
		}
	}
	return output;
}