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
#include <vision.hpp>

using namespace std;
using namespace cv;

class Chroma_processing
{
	public:
				
		Chroma_processing();
		~Chroma_processing();
		
		//image and depth callbacks
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		
		
	private:
	
		ros::NodeHandle nh_;		
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub;
		image_transport::Publisher image_pub;
		image_transport::Publisher image_pub_dif;
			
		string path_;
		string image_topic;
		string image_out_topic;
		string image_out_dif_topic;
		
		
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
