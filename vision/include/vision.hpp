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
#include <opencv2/photo/photo.hpp>
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
		float area 	    = 0.0;
		float ratio     = 0.0;
		float y_norm    = 0.0;
		float distance  = 0.0;
		float depth_std = 0.0;
		
		float x_diff        = 0.0;
		float y_diff        = 0.0;
		float z_diff        = 0.0;
		float z_diff_norm   = 0.0;
		float area_diff     = 0.0;
		float y_norm_diff	= 0.0;
		float ratio_diff    = 0.0;
		float distance_diff = 0.0;
		
		float x_delta   = 0.0;
		float y_delta   = 0.0;
		
	};
	
	struct People 
	{
		vector< Rect_<int> > tracked_boxes;
		vector< float > tracked_rankings;
		vector< Position > tracked_pos;
	};
		
	//Detection and tracking of blobs
	void detectBlobs(const Mat& src, vector< Rect_<int> >& colour_areas, int range, int subsampling, bool detect_people);
	void track(vector< Rect_<int> >& current, People& collection, int width, int height, int rank = 3, int max_rank = 30);
	
	//Depth estimation functions
	float  calculateDepth(const Mat& src, Position& pos);
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
	
	void frameDif(const Mat& src1, const Mat& src2, Mat& dst, float thresh);
		
	//helper functions
	int threshold(Mat& src, Mat& dst, int thresh);
	void gammaCorrection(const Mat& src, float factor);
	void fixRects(vector< Rect_<int> >& rects, int screenW);
	void depthToGray(Mat& src, Mat& dst, float min_depth, float max_depth);
	void grayToDepth(Mat& src, Mat& dst, float max_depth);


#endif
