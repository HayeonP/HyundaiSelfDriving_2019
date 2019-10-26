#include "traffic_sign_classifier.h"

TrafficSignClassifier::TrafficSignClassifier() 
: red(0), green(0), amber(0), none(0)
{
}

TrafficSignClassifier::~TrafficSignClassifier() {

}

int TrafficSignClassifier::detect(Mat src) {
    red = 0, green = 0, amber = 0, none = 0;

	// White Balancing
	src = AutoWhiteBalance(src);
	// Gaussian Filter
	GaussianBlur(src, src, Size(3, 3), 0, 0);

	Mat hsv_img, traffic_sign_img;

	// Get treffic signs by HSV
	cvtColor(src, hsv_img, COLOR_BGR2HSV);

	
	

	traffic_sign_img = trafficSignHSVfilter(hsv_img);
	// imshow("traffic", src);
	// imshow("traffic2", traffic_sign_img);
	waitKey(1);
	
	return color;
}

Mat TrafficSignClassifier::AutoWhiteBalance(Mat src) {
	Mat dst = Mat(src.size(), src.type());

	double YSum = 0;
	double CbSum = 0;
	double CrSum = 0;
	double n = 0;

	double R, B, G, Y, Cb, Cr;
	double a11, a12, a21, a22, b1, b2, Ar, Ab;

	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			int index = i * src.cols* 3 + j * 3;

			unsigned char R = (unsigned char)src.data[index + 2];
			unsigned char G = (unsigned char)src.data[index + 1];
			unsigned char B = (unsigned char)src.data[index];


			Y = 0.299*R + 0.587*G + 0.114*B;
			Cb = -0.1687*R - 0.3313*G + 0.5*B;
			Cr = 0.5*R - 0.4187*G - 0.0813*B;


			if (Y - fabs(Cb) - fabs(Cr) > 100)
			{
				YSum += Y;
				CbSum += Cb;
				CrSum += Cr;
				n++;
			}
		}
	}

	if (n == 0)
	{
		return src;
	}

	YSum /= n;
	CbSum /= n;
	CrSum /= n;

	Y = YSum;
	Cb = CbSum;
	Cr = CrSum;

	a11 = -0.1687*Y - 0.2365*Cr;
	a12 = 0.5*Y + 0.866*Cb;
	a21 = 0.5*Y + 0.701*Cr;
	a22 = -0.0813*Y - 0.1441*Cb;

	b1 = 0.3313*Y - 0.114*Cb - 0.2366*Cr;
	b2 = 0.4187*Y - 0.1441*Cb - 0.299*Cr;


	Ar = (a22*b1 - a12 * b2) / (a11*a22 - a12 * a21);
	Ab = (a21*b1 - a11 * b2) / (a21*a12 - a11 * a22);

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int index = i * src.cols* 3 + j * 3;
			float R = (unsigned char)src.data[index + 2];
			float G = (unsigned char)src.data[index + 1];
			float B = (unsigned char)src.data[index];

			R *= Ar;
			if (R > 255.)
			{
				R = 255.;
			}
			else if (R < 0)
			{
				R = 0.;
			}

			if (G > 255.)
			{
				G = 255.;
			}
			else if (G < 0)
			{
				G = 0.;
			}

			B *= Ab;
			if (B > 255.)
			{
				B = 255.;
			}
			else if (B < 0.)
			{
				B = 0.;
			}

			dst.data[i* dst.cols * 3 + j * 3 + 2] = (unsigned char)R;
			dst.data[i* dst.cols * 3 + j * 3 + 1] = (unsigned char)G;
			dst.data[i* dst.cols * 3 + j * 3] = (unsigned char)B;
		}
	}

	return dst;
}

Mat TrafficSignClassifier::trafficSignHSVfilter(Mat src) {
Mat dst = Mat(src.size(), src.type());
	// HSV
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			int k = INDEX_3CH(j, i, src.cols);

			unsigned char H = src.data[k + 0] * 2;
			unsigned char S = src.data[k + 1];
			unsigned char V = src.data[k + 2];

			//RED
			/*if ((H <= 22 || H >= 330) && S >= 170 && V >= 150) {*/
			if ((H <= 22 || H >= 330) && S >= 170 && V >= 50) {
				dst.data[k + 0] = 0;
				dst.data[k + 1] = 0;
				dst.data[k + 2] = 255;
				red++;
			}
			//GREEN 
			//else if ((H >= 125 && H <= 206) && S >= 100 && V >= 165) {
			else if ((H >= 125 && H <= 206) && S >= 100 && V >= 50) {
				dst.data[k + 0] = 0;
				dst.data[k + 1] = 255;
				dst.data[k + 2] = 0;
				green++;
			}
			//AMBER
			//else if ((H >= 30 && H <= 75) && S >= 127 && V >= 183) {
			else if ((H >= 30 && H <= 75) && S >= 127 && V >= 50) {
				dst.data[k + 0] = 0;
				dst.data[k + 1] = 255;
				dst.data[k + 2] = 255;
				amber++;
			}
			//Background
			else {
				dst.data[k + 0] = 0;
				dst.data[k + 1] = 0;
				dst.data[k + 2] = 0;
				none++;
			}

		}
	}

    
	if(red + green + amber == 0 ) color = none;
	else{
		color = max(red, green);
    	color = max(color, amber);
		if(color == red) color = RED;
		else if (color == green) color = GREEN;
		else if (color == amber) color = AMBER;
		else color = none;
	}
	return dst;
}
