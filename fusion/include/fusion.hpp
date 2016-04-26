#ifndef FUSION_HPP
#define FUSION_HPP
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
#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <utility.hpp>
#include <stdafx.h>
#include <dataanalysis.h>

using namespace std;
using namespace cv;
using namespace alglib;

#define DEPTH_MAX 6000.0  /**< Default maximum distance. Only use this for initialization. */
#define DEPTH_MIN 0.0  /**< Default minimum distance. Only use this for initialization. */

struct timeval tv;
time_t curtime;
char timeBuf[80];

class Fusion_processing
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
		
		
		
		Fusion_processing();		  
		~Fusion_processing();
				
		
		void callback(const sensor_msgs::Image::ConstPtr& chroma_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::Image::ConstPtr& depth_dif_msg);
		
		//Detection and tracking of blobs
		void detectBlobs(Mat& src, vector< Rect_<int> >& colour_areas, int range);
		void track(vector< Rect_<int> >& current, People& collection, int rank = 3, float threshold = 0.2);
		
		//Depth estimation functions
		double calculateDepth(Mat& src, Rect_<int> personRect);
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
		kmeansreport clusterize(const vector<float>& vec, int clusters);
		void writeCSV(Fusion_processing::People& collection, string path);
		
		
	private:
	
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		typedef image_transport::SubscriberFilter ImageSubscriber;
				
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > *sync;
  	
		string path_;
		string session_path;
		string image_topic;
		string depth_topic;
		string depth_dif_topic;
		
		vector< Rect_<int> > depth_rects;
		
		People people;
		
		bool playback_topics;
		bool write_csv;
		bool display;
		bool has_image = false;
		
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
