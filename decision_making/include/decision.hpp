#ifndef DECISION_HPP
#define DECISION_HPP
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <limits>
#include <exception>
#include <sstream>  
#include <boost/filesystem.hpp>

#include <fusion/FusionMsg.h>
#include <fusion/Box.h>
#include <decision_making/Event.h>

using namespace std;

#define DEPTH_MAX 6000.0  /**< Default maximum distance. Only use this for initialization. */
#define DEPTH_MIN 0.0  /**< Default minimum distance. Only use this for initialization. */

class Decision_making
{
	public:
		
        
        struct Position 
        {
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;
            float top = 0.0;
            float height = 0.0;
            float distance = 0.0;
            float acc_distance = 0.0;
        };	
        
        struct Rectangle 
        {
            
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;
            float width = 0.0;
            float height = 0.0;
        };
        
        struct Box 
        {
            int id;
            Position pos;
            Rectangle rect;
        };	
		
        Decision_making();	
		
        void callback(const fusion::FusionMsg::ConstPtr& msg);
        
        float median(vector<float> values);
        string create_directory(string path_, bool create_csv, vector<string> fields);
        string getTime(string format);



    private:
        ros::NodeHandle nh_;
        ros::Subscriber fusion_sub;
        ros::Publisher results_publisher;
		string camera_frame;
        string path_;
        string session_path;
        string results_topic;
        string events_topic;
        string csv_fields;
        bool display;
        bool create_dir;
        bool write_csv;
		struct tm gmtm;
        int frameCounter = -1;
        int history_size;
        ros::Time standUp_time = ros::Time(0);
            
        vector<Box> boxes;
        vector<vector<Box>> boxes_hist;
        vector<bool> inserted;
        vector<vector<float>> distances;
        vector<vector<Position>> positions;
        double max_depth;
        double min_depth;
	
};

int main(int argc, char** argv);

#endif
