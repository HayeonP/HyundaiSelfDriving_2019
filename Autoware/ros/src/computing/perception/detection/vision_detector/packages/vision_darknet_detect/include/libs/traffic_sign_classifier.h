#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string.h>

#define PRINT 0

#define NONE		    -1
#define RED			    0
#define GREEN		    1
#define AMBER		    2

#define INDEX_3CH(X, Y, W)		(Y) * W * 3 + (X) * 3
#define INDEX_1CH(X, Y, W)		(Y) * W + (X)

using namespace cv;
using namespace std;

#pragma once

class TrafficSignClassifier {

public:
	TrafficSignClassifier();
	~TrafficSignClassifier();

    int red=0, green=0, amber=0, none=0;
    int color;

	int detect(Mat src);

	Mat AutoWhiteBalance(Mat src);
	Mat trafficSignHSVfilter(Mat src);
};

