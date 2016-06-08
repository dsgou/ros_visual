#ifndef DECISION_HPP
#define DECISION_HPP
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <limits>
#include <exception>

#include <fusion/FusionMsg.h>
#include <fusion/Box.h>

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

    private:
        ros::NodeHandle nh_;
        ros::Subscriber fusion_sub;
        string path_;
        string session_path;
        string results_topic;
        bool display;
        int frameCounter = -1;
            
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
