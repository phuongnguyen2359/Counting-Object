/*
 * detect.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: JackD
 */

#include "detect.h"

detect::detect(std::vector<cv::Point> _contour) {
    currentContour = _contour;
    currentBounding = boundingRect(currentContour);

    cv::Point currentCenter;

    currentCenter.x = (currentBounding.x + currentBounding.x + currentBounding.width) / 2;
    currentCenter.y = (currentBounding.y + currentBounding.y + currentBounding.height) / 2;

    centerPositions.push_back(currentCenter);

    currentDiagonalSize = sqrt(pow(currentBounding.width, 2) + pow(currentBounding.height, 2));
    currentAspectRatio = (float)currentBounding.width / (float)currentBounding.height;

    stillTracked = true;
    currentMatchFoundOrNewDetect = true;

    numOfConsecutiveFramesWithoutAMatch = 0;
}

void detect::predictNextPosition(void) {
    int numPositions = (int)centerPositions.size();

    if (numPositions == 1) {
        predictedNextPosition.x = centerPositions.back().x;
        predictedNextPosition.y = centerPositions.back().y;
    }
    else if (numPositions == 2) {
        int pointX = centerPositions[1].x - centerPositions[0].x;
        int pointY = centerPositions[1].y - centerPositions[0].y;

        predictedNextPosition.x = centerPositions.back().x + pointX;
        predictedNextPosition.y = centerPositions.back().y + pointY;
    }
    else if (numPositions == 3) {
        int sumX = ((centerPositions[2].x - centerPositions[1].x) * 2) + ((centerPositions[1].x - centerPositions[0].x) * 1);
        int sumY = ((centerPositions[2].y - centerPositions[1].y) * 2) + ((centerPositions[1].y - centerPositions[0].y) * 1);

        int pointX = (int)round((float)sumX / 3.0);
        int pointY = (int)round((float)sumY / 3.0);

        predictedNextPosition.x = centerPositions.back().x + pointX;
        predictedNextPosition.y = centerPositions.back().y + pointY;
    }
    else if (numPositions == 4) {
        int sumX = ((centerPositions[3].x - centerPositions[2].x) * 3) + ((centerPositions[2].x - centerPositions[1].x) * 2) + ((centerPositions[1].x - centerPositions[0].x) * 1);
        int sumY = ((centerPositions[3].y - centerPositions[2].y) * 3) + ((centerPositions[2].y - centerPositions[1].y) * 2) + ((centerPositions[1].y - centerPositions[0].y) * 1);

        int pointX = (int)round((float)sumX / 6.0);
        int pointY = (int)round((float)sumY / 6.0);

        predictedNextPosition.x = centerPositions.back().x + pointX;
        predictedNextPosition.y = centerPositions.back().y + pointY;
    }
    else if (numPositions >= 5) {
        int sumX = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) * 4) + ((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) * 3) + ((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) * 2) + ((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) * 1);
        int sumY = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) * 4) + ((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) * 3) + ((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) * 2) + ((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) * 1);

        int pointX = (int)round((float)sumX / 10.0);
        int pointY = (int)round((float)sumY / 10.0);

        predictedNextPosition.x = centerPositions.back().x + pointX;
        predictedNextPosition.y = centerPositions.back().y + pointY;
    }
}
