#ifndef VISION_HPP
#define VISION_HPP
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <limits>
#include <exception>
#include <opencv2/core/core.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;
	
	
	
	struct Position 
	{
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;
		float top = 0.0;
		float height = 0.0;
		float distance = 0.0;
	};
	
	struct People 
	{
		vector< Rect_<int> > tracked_boxes;
		vector< float > tracked_rankings;
		vector< Position > tracked_pos;
	};
		
	//Detection and tracking of blobs
	void detectBlobs(Mat& src, vector< Rect_<int> >& colour_areas, int range, bool detect_people);
	void track(vector< Rect_<int> >& current, People& collection, int rank = 3, float threshold = 0.2);
	
	//Depth estimation functions
	double calculateDepth(Mat& src, Rect_<int> personRect);
	double minDepth(vector<double> vec, int number);
	double centerDepth(const Mat& src, int number);
	double combineDepth(double saveMin, double saveCenter, double saveCluster, double min_depth = 0.0, double max_depth = 6.0);
		
	//Position estimation
	void calculatePosition(Rect& rect, Position& pos, int width = 640, int height = 480, int Hfield = 58, int Vfield = 45);
	
	//Region growing algorithms
	void upVerticalFill(Mat& src, float threshold, bool flag);
	void upVerticalFill2(Mat& src, float threshold, bool scale);
	void rightHorizontalFill(Mat& cur_Mat, float threshold, bool scale);
	void rectFill(Mat& cur_Mat, float threshold, int range);
	
	//Background & foreground estimation, to be used in sequence
	void estimateBackground(Mat& src, Mat& dst, vector<Mat>& storage, int recursion, float ratio = 0.04, int index = 0);
	void estimateForeground(Mat& src1, Mat& src2, Mat& dst);
	
	void frameDif(Mat& src1, Mat& src2, Mat& dst, float threshold);
		
	//helper functions
	int threshold(Mat& src, Mat& dst, int thresh);
	void gammaCorrection(Mat& src);
	void fixRects(vector< Rect_<int> >& rects, int screenW);
	void depthToGray(Mat& src, Mat& dst, float min_depth, float max_depth);
	void grayToDepth(Mat& src, Mat& dst, float max_depth);


#endif
