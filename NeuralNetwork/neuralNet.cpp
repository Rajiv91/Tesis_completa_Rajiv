#include <iostream> 
#include <cstdlib>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "time.h"
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
//#include "ml.h"


using namespace std;
using namespace cv;

int numTrainingPoints=200;
int numTestPoints=2000;
int size=200;
int eq=3;


float evaluate(Mat& predicted, Mat& actual) {
	assert(predicted.rows == actual.rows);
	int t = 0;
	int f = 0;
	for(int i = 0; i < actual.rows; i++) {
		float p = predicted.at<float>(i,0);
		float a = actual.at<float>(i,0);
		if((p >= 0.0 && a >= 0.0) || (p <= 0.0 &&  a <= 0.0)) {
			t++;
		} else {
			f++;
		}
	}
	return (t * 1.0) / (t + f);
}

// function to learn
int f(float x, float y, int equation) {
	switch(equation) {
	case 0:
		return y > sin(x*10) ? -1 : 1;
		break;
	case 1:
		return y > cos(x * 10) ? -1 : 1;
		break;
	case 2:
		return y > 2*x ? -1 : 1;
		break;
	case 3:
		return y > tan(x*10) ? -1 : 1;
		break;
	default:
		return y > cos(x*10) ? -1 : 1;
	}
}

Mat labelData(Mat points, int equation) {
	Mat labels(points.rows, 1, CV_32FC1);
	for(int i = 0; i < points.rows; i++) {
			 float x = points.at<float>(i,0);
			 float y = points.at<float>(i,1);
			 labels.at<float>(i, 0) = f(x, y, equation);
		}
	return labels;
}

void plot_binary(Mat& data, Mat& classes, string name) {
	Mat plot(size, size, CV_8UC3);
	plot.setTo(Scalar(255.0,255.0,255.0));
	for(int i = 0; i < data.rows; i++) {

		float x = data.at<float>(i,0) * size;
		float y = data.at<float>(i,1) * size;

		if(classes.at<float>(i, 0) > 0) {
			circle(plot, Point(x,y), 2, CV_RGB(255,0,0),1);
		} else {
			circle(plot, Point(x,y), 2, CV_RGB(0,255,0),1);
		}
	}
	imshow(name, plot);
        waitKey(0);
}

int main(int argc, char **argv)
{
  CvANN_MLP mlp;
  
  CvTermCriteria criteria;
  CvANN_MLP_TrainParams params;
  criteria.max_iter = 100;
  criteria.epsilon = 0.00001f;
  criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
  params.train_method = CvANN_MLP_TrainParams::BACKPROP;
  params.bp_dw_scale = 0.05f;
  params.bp_moment_scale = 0.05f;
  params.term_crit = criteria;

  //Topología de la red
  Mat layers= Mat(4,1, CV_32SC1);
  layers.row(0) = cv::Scalar(2);
  layers.row(1) = cv::Scalar(10);
  layers.row(2) = cv::Scalar(25);
  layers.row(3) = cv::Scalar(1);
  mlp.create(layers);


  Mat trainingData(numTrainingPoints, 2, CV_32FC1);
  Mat testData(numTestPoints, 2, CV_32FC1);
  randu(trainingData,0,1);
  randu(testData,0,1);
  Mat trainingClasses = labelData(trainingData, eq);
  Mat testClasses = labelData(testData, eq);
  
  mlp.train(trainingData, trainingClasses, Mat(), Mat(), params);

  Mat response(1, 1, CV_32FC1);
  Mat predicted(testClasses.rows, 1, CV_32F);
  for(int i = 0; i < testData.rows; i++)
  {
          Mat response(1, 1, CV_32FC1);
          Mat sample = testData.row(i);

          mlp.predict(sample, response);
          predicted.at<float>(i,0) = response.at<float>(0,0);

  }

  cout << "Accuracy_{MLP} = " << evaluate(predicted, testClasses) << endl;
  plot_binary(testData, predicted, "Predictions Backpropagation");

  cout<<"hola"<<endl;





  return 0;
}



