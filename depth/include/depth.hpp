#ifndef DEPTH_HPP
#define DEPTH_HPP_HPP
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <opencv2/core/core.hpp>
#include <limits>
#include <opencv2/video/background_segm.hpp>
#include <exception>

#include <stdafx.h>
#include <dataanalysis.h>

using namespace std;
using namespace cv;
using namespace alglib;

#define DEPTH_MAX 6000.0  /**< Default maximum distance. Only use this for initialization. */
#define DEPTH_MIN 0.0  /**< Default minimum distance. Only use this for initialization. */

class Depth_processing
{
	public:
		
		struct Position 
		{
			float x = 0.0;
			float y = 0.0;
			float z = 0.0;
			float height = 0.0;
			float distance = 0.0;
		};
		
		struct People 
		{
			vector< Rect_<int> > tracked_boxes;
			vector< float > tracked_rankings;
			vector< Position > tracked_pos;
		};
		
		
		Depth_processing();
		~Depth_processing();
		
		//depth callback
		void depthCb(const sensor_msgs::ImageConstPtr& msg);
		
		//Region growing algorithms
		void upVerticalFill(Mat& src, float threshold, bool flag);
		void upVerticalFill2(Mat& src, float threshold, bool scale);
		void rightHorizontalFill(Mat& cur_Mat, float threshold, bool scale);
		void rectFill(Mat& cur_Mat, float threshold, int range);
		
		//Background & foreground estimation, to be used in sequence
		void estimateBackground(Mat& src, Mat& dst, vector<Mat>& storage, int recursion, float ratio = 0.04, int index = 0);
		void estimateForeground(Mat& src1, Mat& src2, Mat& dst);
		
		//Detection and tracking of blobs
		void detectBlobs(Mat& src, vector< Rect_<int> >& colour_areas, int range);
		void track(vector< Rect_<int> >& current, People& collection, int rank = 3, float threshold = 0.2);
		
		//Depth estimation functions
		double calculateDepth(const Mat& src);
		double minDepth(vector<double> vec, int number);
		double centerDepth(const Mat& src, int number);
		double combineDepth(double saveMin, double saveCenter, double saveCluster, double min_depth = 0.0, double max_depth = 6.0);
		
		//Position estimation
		void calculatePosition(Rect& rect, Position& pos);
		
		//helper functions
		int threshold(Mat& src, Mat& dst, int thresh);
		void fixRects(vector< Rect_<int> >& rects, int screenW);
		void depthToGray(Mat& src, Mat& dst, float min_depth, float max_depth);
		void grayToDepth(Mat& src, Mat& dst, float max_depth);
		void frameDif(Mat& src1, Mat& src2, Mat& dst, float threshold);
		kmeansreport clusterize(const vector<float>& vec, int clusters);
		
		
	private:
	
		ros::NodeHandle nh_;
		cv_bridge::CvImagePtr cv_ptr;
		image_transport::ImageTransport it_;
		image_transport::Subscriber depth_sub;
		image_transport::Publisher  depth_pub;
		image_transport::Publisher  depth_pub_dif;
			
		string path_;
		string depth_topic;
		string depth_out_image_topic;
		string depth_out_dif_topic;
		
		People people;
		
		Mat ref_depth;
		Mat dif_depth;
		
		vector< Rect_<int> > depth_rects;
		
		
		bool playback_topics;
		bool display;
		
		int depth_width = 640;
		int depth_height = 480;
		int Hfield = 58;
		int Vfield = 45;
		int interval = 5;
		int frameCounter = -1, dFrameCounter = -1, myThreshold = 100;
		int depthCounter = -1;
		int range = 2; //in pixels
		int verRange = 7; //in pixels
		int recR = 2;
		
		float backFactor = 0.40;
		
		double max_depth;
		double min_depth;
		double horThreshold = 0.33;
		double vertThreshold = 0.5;
		double recThreshold = 0.3;
		double all, curAll, refAll;
		
		
		long curTime ;
		
		vector<Mat> depth_storage;
		
		Mat back_Mat;
	
};

int main(int argc, char** argv);
#endif
