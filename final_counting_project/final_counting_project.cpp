//============================================================================
// Name        : final_counting_project.cpp
// Author      : JackD
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include "detect.h"

using namespace cv;

#define SHOW_STEPS

// global variables
const Scalar black = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar white = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar yellow = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar green = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar red = cv::Scalar(0.0, 0.0, 255.0);
const cv::Scalar grey = cv::Scalar(169.0, 169.0, 169.0);
VideoCapture capVideo;
int carCount = 0;
int truckCount = 0;
int bikeCount = 0;
int imageCount = 1;

// functions
void matchCurrentDetectsToExistingDetects(std::vector<detect> &existingDetects, std::vector<detect> &currentDetects);
void addDetectToExistingDetects(detect &currentDetect, std::vector<detect> &existingDetects, int &index);
void addNewDetect(detect &currentDetect, std::vector<detect> &existingDetects);
double distance(Point point_1, Point point_2);
bool crossedTheLine(std::vector<detect> &detects, int &linePosition);
void drawDetectRect(std::vector<detect> &detects, Mat &frame2Copy);
void drawCarCount(Mat &frame2Copy);
void captureCar(Rect rectPosition);
void captureTruck(Rect rectPosition);
void captureBike(Rect rectPosition);
void saveImage(Mat savedImage);
bool addGit(char * repo, char * sign, const char * content, int content_sz, const char * message);
void commitGit(char * repo, char * sign, const char * content, const char * message);
void pushGit(char gitAccount[50], char gitPass[50]);

int main(void) {
	Mat frame_1;
	Mat frame_2;
	Point crossingLine[2];

	std::vector<detect> detects;
	capVideo.open("video.mp4");

	bool firstFrame = true;

	char escKey = 0;
	int frameCount = 2;

	if (!capVideo.isOpened()) {
		std::cout << "error reading video file" << std::endl << std::endl;
		_getch();
		return (0);
	}
	else if (capVideo.get(CV_CAP_PROP_FRAME_COUNT) < 2) {
		std::cout << "error: video file must have at least two frames";
		_getch();
		return (0);
	}

	capVideo.read(frame_1);
	capVideo.read(frame_2);

	int linePosition = (int)round((double)frame_1.rows * 0.35);

	crossingLine[0].x = 420;
	crossingLine[0].y = linePosition;
	crossingLine[1].x = frame_1.cols - 370;
	crossingLine[1].y = linePosition;

	while (capVideo.isOpened() && escKey != 27) {
		std::vector<detect> currentDetects;
		std::vector<std::vector<Point> > contours;

		Mat frame1Copy = frame_1.clone();
		Mat frame2Copy = frame_2.clone();
		Mat difference;
		Mat thresh;

		cvtColor(frame1Copy, frame1Copy, CV_BGR2GRAY);
		cvtColor(frame2Copy, frame2Copy, CV_BGR2GRAY);

		GaussianBlur(frame1Copy, frame1Copy, Size(5, 5), 0);
		GaussianBlur(frame2Copy, frame2Copy, Size(5, 5), 0);

		absdiff(frame1Copy, frame2Copy, difference);

		threshold(difference, thresh, 30, 255.0, CV_THRESH_BINARY);

		Mat structuringElement = getStructuringElement(MORPH_RECT, Size(5, 5));

		for (unsigned int i = 0; i < 2; i++) {
			dilate(thresh, thresh, structuringElement);
			dilate(thresh, thresh, structuringElement);
			erode(thresh, thresh, structuringElement);
		}

		Mat threshCopy = thresh.clone();

		findContours(threshCopy, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		std::vector<std::vector<Point> > convexHulls(contours.size());

		for (unsigned int i = 0; i < contours.size(); i++) {
			convexHull(contours[i], convexHulls[i]);
		}

		for (auto &convexHull : convexHulls) {
			detect possibleDetect(convexHull);

			if (possibleDetect.currentBounding.area() > 400 && possibleDetect.currentAspectRatio > 0.2 && possibleDetect.currentAspectRatio < 4.0
					&& possibleDetect.currentBounding.width > 30 && possibleDetect.currentBounding.height > 30 && possibleDetect.currentDiagonalSize > 60.0
					&& (contourArea(possibleDetect.currentContour) / (double) possibleDetect.currentBounding.area()) > 0.50) {
				currentDetects.push_back(possibleDetect);
			}
		}

		if (firstFrame == true) {
			for (auto &currentDetect : currentDetects) {
				detects.push_back(currentDetect);
			}
		}
		else {
			matchCurrentDetectsToExistingDetects(detects, currentDetects);
		}

		frame2Copy = frame_2.clone();
		drawDetectRect(detects, frame2Copy);

		bool oneCrossedTheLine = crossedTheLine(detects, linePosition);

		if (oneCrossedTheLine == true) {
			line(frame2Copy, crossingLine[0], crossingLine[1], green, 2);
		}
		else {
			line(frame2Copy, crossingLine[0], crossingLine[1], red, 2);
		}

		drawCarCount(frame2Copy);
		imshow("Counting Vehicle", frame2Copy);

		currentDetects.clear();
		frame_1 = frame_2.clone();

		if (capVideo.read(frame_2) == NULL) {
			capVideo.open("video.mp4");
			capVideo.read(frame_2);
		}

		firstFrame = false;
		frameCount++;
		escKey = waitKey(1);
	}

	if (escKey != 27) {
		waitKey(0);
	}

	return 0;
}

void matchCurrentDetectsToExistingDetects(std::vector<detect> &existingDetects, std::vector<detect> &currentDetects) {
	for (auto &existingDetect : existingDetects) {
		existingDetect.currentMatchFoundOrNewDetect = false;
		existingDetect.predictNextPosition();
	}

	for (auto &currentDetect : currentDetects) {
		int indexOfLeastDistance = 0;
		double leastDistance = 100000.0;

		for (unsigned int i = 0; i < existingDetects.size(); i++) {
			if (existingDetects[i].stillTracked == true) {
				double dblDistance = distance(currentDetect.centerPositions.back(), existingDetects[i].predictedNextPosition);

				if (dblDistance < leastDistance) {
					leastDistance = dblDistance;
					indexOfLeastDistance = i;
				}
			}
		}

		if (leastDistance < currentDetect.currentDiagonalSize * 0.5) {
			addDetectToExistingDetects(currentDetect, existingDetects, indexOfLeastDistance);
		}
		else {
			addNewDetect(currentDetect, existingDetects);
		}
	}

	for (auto &existingDetect : existingDetects) {
		if (existingDetect.currentMatchFoundOrNewDetect == false) {
			existingDetect.numOfConsecutiveFramesWithoutAMatch++;
		}

		if (existingDetect.numOfConsecutiveFramesWithoutAMatch >= 5) {
			existingDetect.stillTracked = false;
		}
	}
}

void addDetectToExistingDetects(detect &currentDetect, std::vector<detect> &existingDetects, int &index) {
	existingDetects[index].currentContour = currentDetect.currentContour;
	existingDetects[index].currentBounding = currentDetect.currentBounding;
	existingDetects[index].centerPositions.push_back(currentDetect.centerPositions.back());
	existingDetects[index].currentDiagonalSize = currentDetect.currentDiagonalSize;
	existingDetects[index].currentAspectRatio = currentDetect.currentAspectRatio;

	existingDetects[index].stillTracked = true;
	existingDetects[index].currentMatchFoundOrNewDetect = true;
}

void addNewDetect(detect &currentDetect, std::vector<detect> &existingDetects) {
	currentDetect.currentMatchFoundOrNewDetect = true;
	existingDetects.push_back(currentDetect);
}

double distance(Point point_1, Point point_2) {
    int intX = abs(point_1.x - point_2.x);
    int intY = abs(point_1.y - point_2.y);

    return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

bool crossedTheLine(std::vector<detect> &detects, int &linePosition) {
    bool crossedTheLine = false;

    for (auto detect : detects) {
        if (detect.stillTracked == true && detect.centerPositions.size() >= 2) {
            int previousFrameIndex = (int)detect.centerPositions.size() - 2;
            int currentFrameIndex = (int)detect.centerPositions.size() - 1;

            if (detect.centerPositions[previousFrameIndex].y > linePosition && detect.centerPositions[currentFrameIndex].y <= linePosition
            		&& detect.centerPositions[previousFrameIndex].x > 200 && detect.centerPositions[previousFrameIndex].x < 980
					&& detect.centerPositions[currentFrameIndex].x > 200 && detect.centerPositions[currentFrameIndex].x < 980) {
                crossedTheLine = true;

                if (detect.currentBounding.width < 60) {
                	bikeCount++;
                	captureBike(detect.currentBounding);
                                }
                else if (detect.currentBounding.height <= 95 && detect.currentBounding.width > 60) {
                	carCount++;
                	captureCar(detect.currentBounding);
                }
                else if (detect.currentBounding.height > 95) {
                	truckCount++;
                	captureTruck(detect.currentBounding);
                }
            }
        }
    }

    return crossedTheLine;
}

void captureCar(Rect rectPosition) {
	Mat image;
	Mat croped;
	Rect cropSize;

	capVideo.read(image);
	cropSize = rectPosition;
	croped = image(cropSize);

	namedWindow("Car Detection", CV_WINDOW_FREERATIO);
	imshow("Car Detection", croped);
	saveImage(croped);
}

void captureTruck(Rect rectPosition) {
	Mat image;
	Mat croped;
	Rect cropSize;

	capVideo.read(image);
	cropSize = rectPosition;
	croped = image(cropSize);

	namedWindow("Truck Detection", CV_WINDOW_FREERATIO);
	imshow("Truck Detection", croped);
	saveImage(croped);
}

void captureBike(Rect rectPosition) {
	Mat image;
	Mat croped;
	Rect cropSize;

	capVideo.read(image);
	cropSize = rectPosition;
	croped = image(cropSize);

	namedWindow("Bike Detection", CV_WINDOW_FREERATIO);
	imshow("Bike Detection", croped);
	saveImage(croped);
}

void saveImage(Mat savedImage) {
	Mat bg;
	Mat bg_2;
	bg = imread("bg.png");
	bg_2 = imread("bg2.png");

	savedImage.copyTo(bg(Rect(0, 0, savedImage.cols, savedImage.rows)));
	bg.copyTo(bg_2(Rect(3, 3, bg.cols, bg.rows)));
	char fileName[100];
	sprintf(fileName, "g%d.jpg", imageCount);  // make the file name
	imwrite(fileName, bg_2);  // save as a JPEG file
	imageCount ++;
}


void drawDetectRect(std::vector<detect> &detects, Mat &frame2Copy) {
    for (unsigned int i = 0; i < detects.size(); i++) {
        if (detects[i].stillTracked == true) {
            rectangle(frame2Copy, detects[i].currentBounding, red, 2);
        }
    }
}

void drawCarCount(Mat &frame2Copy) {
	Mat boder;
	boder = imread("boder.png");
	boder.copyTo(frame2Copy(Rect(0, 660, boder.cols, boder.rows)));

    int fontFace = CV_FONT_HERSHEY_SIMPLEX;
    int fontScale = 1;
    int fontThickness = (int)round(fontScale * 1.5);
    char line_1[100];
    char line_2[100];
    char line_3[100];
    char name[100];

    Size textSize(5, 5);
    Point textPosition_1;
    Point textPosition_2;
    Point textPosition_3;

	textPosition_1.x = 140;
	textPosition_1.y = 700;
	textPosition_2.x = 520;
	textPosition_2.y = 700;
	textPosition_3.x = 890;
	textPosition_3.y = 700;

	sprintf(line_1, "%d", carCount);
	sprintf(line_2, "%d", truckCount);
	sprintf(line_3, "%d", bikeCount);
	sprintf(name, "Engineering Computing - Vladimir Mariano - HPH - Counting Vehicle");
	putText(frame2Copy, line_1, textPosition_1, fontFace, fontScale, green,
			fontThickness);
	putText(frame2Copy, line_2, textPosition_2, fontFace, fontScale, green,
			fontThickness);
	putText(frame2Copy, line_3, textPosition_3, fontFace, fontScale, green,
			fontThickness);
	putText(frame2Copy, name, Point(10, 40), fontFace, fontScale, red,
				fontThickness);
}

bool addGit(char * repo, char * sign, const char * content, int content_sz, const char * message) {
	int rc;
	bool b = false;

	char gitAccount[50] = "pjit" ;
	char gitPass[50] = "pjit123";

	if (rc == 0) {
		commitGit(repo, sign, content, message);
		pushGit(gitAccount, gitPass);
	}
	return b;
}

void commitGit(char * repo, char * sign, const char * content, const char * message) {}
void pushGit(char gitAccount[50], char gitPass[50]) {}

