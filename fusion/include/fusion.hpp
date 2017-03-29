#ifndef FUSION_HPP
#define FUSION_HPP
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <limits>
#include <exception>

#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <utility.hpp>
#include <vision.hpp>

#include <ros_visual_msgs/FusionMsg.h>
#include <ros_visual_msgs//Box.h>

using namespace std;
using namespace cv;

#define DEPTH_MAX 6000.0  /**< Default maximum distance. Only use this for initialization. */
#define DEPTH_MIN 0.0  /**< Default minimum distance. Only use this for initialization. */

class Fusion_processing
{
	public:
			
		Fusion_processing();		  
		~Fusion_processing();
				
		void chromaCb(const sensor_msgs::ImageConstPtr& msg);
		void depthCb(const sensor_msgs::ImageConstPtr& msg);

		void writeCSV(People& collection, string path, ros::Time time);
		void publishResults(People& collection, ros::Time time);

		
		
	private:
	
		ros::NodeHandle nh_;
		ros::Publisher results_publisher;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub;
		image_transport::Subscriber depth_sub;
		ros::Time previous_time;
  	
		Mat depth_Mat;
		Mat back_Mat;
		vector<Mat> depth_storage;
		vector< Rect_<int> > depth_rects;
		
		string path_;
		string session_path;
		string image_topic;
		string image_dif_topic;
		string depth_topic;
		string results_topic;
        string csv_fields;
		string camera_frame;
		
		People people;
		
		bool playback_topics;
		bool display;
		bool create_directory;
		bool write_csv;
		bool has_image = false;
		bool depth_available = false;
		
		int Hfield 		  = 58;
		int Vfield 		  = 45;
		int depth_width   = 640;
		int depth_height  = 480;
		int interval 	  = 5;
		int frameCounter  = -1; 
		int dFrameCounter = -1;
		int myThreshold	  = 100;
		int depthCounter  = -1;
		int range 		  = 2; //in pixels
		int verRange 	  = 7; //in pixels
		int recR 		  = 2;
		int counter = 0;
		int max_rank = 0;
		long curTime ;
		float backFactor = 0.40;
		
		double all;
		double curAll;
		double refAll;
		double max_depth;
		double min_depth;
		double horThreshold  = 0.33;
		double vertThreshold = 0.5;
		double recThreshold  = 0.3;
		
		
	
};

int main(int argc, char** argv);

#endif
