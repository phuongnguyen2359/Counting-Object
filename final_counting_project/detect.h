/*
 * detect.h
 *
 *  Created on: Aug 16, 2016
 *      Author: JackD
 */

#ifndef DETECT_H_
#define DETECT_H_

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

class detect {
public:
	// variables
	std::vector<cv::Point> currentContour;
	std::vector<cv::Point> centerPositions;
	cv::Rect currentBounding;
	cv::Point predictedNextPosition;

	double currentDiagonalSize;
	double currentAspectRatio;

	int numOfConsecutiveFramesWithoutAMatch;

	bool currentMatchFoundOrNewDetect;
	bool stillTracked;

	// functions
	detect(std::vector<cv::Point> _contour);
	void predictNextPosition(void);
};

#endif /* DETECT_H_ */
