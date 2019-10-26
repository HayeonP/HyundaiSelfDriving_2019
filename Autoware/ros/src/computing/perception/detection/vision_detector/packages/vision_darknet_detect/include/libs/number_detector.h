#pragma once
#include <opencv2/opencv.hpp>
#include "blobLabeling.h"
#include <ml.h>
//#include "yolo_darknet_detect_def.h"

#define NUMBER_DETECT_FLAG 0
#define SHOW_FLAG 0

using namespace std;
using namespace cv;

class NumberDetector {
private:
	IplImage *_Number[10];
	cv::Ptr<cv::ml::ANN_MLP> nnetwork;
	int resultNum[3][2];
	int digit;
	std::string network_path_;
private:	
	int annPredict(IplImage* roi);
	void BubbleSorting(int num[3][2]);
	
	

public:
	NumberDetector(string network_path);
	NumberDetector() {};
	~NumberDetector();
	int detect(Mat input);
	void setPath(std::string path) {network_path_ = path;};
	
};
