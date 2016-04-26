#ifndef CHROMA_HPP
#define CHROMA_HPP
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

using namespace std;
using namespace cv;

class Chroma_processing
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
		
		
		Chroma_processing();
		~Chroma_processing();
		
		//image and depth callbacks
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
				
		//Background & foreground estimation, to be used in sequence
		void estimateBackground(Mat& src, Mat& dst, vector<Mat>& storage, int recursion, float ratio = 0.04, int index = 0);
		void estimateForeground(Mat& src1, Mat& src2, Mat& dst);
		
		//Detection and tracking of blobs
		void detectBlobs(const Mat& src, vector< Rect_<int> >& colour_areas, int range);
		void track(vector< Rect_<int> >& current, People& collection, int rank = 3, float threshold = 0.2);
				
		//helper functions
		int threshold(Mat& src, Mat& dst, int thresh);
		inline void gammaCorrection(Mat& src);
		void frameDif(Mat& src1, Mat& src2, Mat& dst, float threshold);
		
		
	private:
	
		ros::NodeHandle nh_;
		cv_bridge::CvImagePtr cv_ptr;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub;
		image_transport::Publisher image_pub;
			
		string path_;
		string image_topic;
		string image_out_topic;
		
		
		Mat cur_rgb;
		Mat ref_rgb;
		Mat dif_rgb;
		
		vector< Rect_<int> > rgb_rects;
		
		
		bool playback_topics;
		bool display;
		bool has_image = false;
		
		int interval = 5;
		int frameCounter = -1, myThreshold = 100;
		
		float backFactor = 0.40;
		
		long curTime ;
		
		vector<Mat> rgb_storage;
		
		Mat back_Mat;
	
};

int main(int argc, char** argv);
#endif
