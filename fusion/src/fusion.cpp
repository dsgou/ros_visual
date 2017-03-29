#include <fusion.hpp>


Fusion_processing::Fusion_processing()
: it_(nh_)
{
	 //Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("camera_frame" 	 , camera_frame		, string("camera_link"));
	local_nh.param("results_topic"	 , results_topic	, string("results"));
	local_nh.param("image_topic"	 , image_topic		, string("/chroma_proc/image"));
	local_nh.param("image_dif_topic" , image_dif_topic  , string("/chroma_proc/image_dif"));
	local_nh.param("depth_topic"     , depth_topic		, string("/depth_proc/image"));
	local_nh.param("project_path"	 , path_ 			, string(""));
	local_nh.param("csv_fields"		 , csv_fields 		, string(""));
	local_nh.param("playback_topics" , playback_topics  , false);
	local_nh.param("create_directory", create_directory , false);
	local_nh.param("write_csv"		 , write_csv 		, false);
	local_nh.param("display"		 , display 			, false);
	local_nh.param("max_depth"		 , max_depth 		, DEPTH_MAX);
	local_nh.param("min_depth"		 , min_depth 		, DEPTH_MIN);
	local_nh.param("fps"			 , max_rank 		, 30);
	
	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Fusion_processing","Subscribing at compressed topics \n"); 
		depth_sub = it_.subscribe(depth_topic, 1, &Fusion_processing::depthCb, this, image_transport::TransportHints("compressed"));
    } 
    else
    {
		depth_sub = it_.subscribe(depth_topic, 1, &Fusion_processing::depthCb, this);
	}
	
    image_sub = it_.subscribe(image_dif_topic, 1, &Fusion_processing::chromaCb, this);
    
    
    results_publisher = local_nh.advertise<ros_visual_msgs::FusionMsg>(results_topic, 1);
	
	if(create_directory)
    {
        string temp;
        vector<string> fields;
        stringstream stream(csv_fields);
        while(stream >> temp)
        {
			replace(temp.begin(), temp.end(), ',', ' ');
            fields.push_back(temp);
        }
        Utility u;
		session_path = u.create_directory(path_, write_csv, fields, false);
    }
}

Fusion_processing::~Fusion_processing()
{
	//destroy GUI windows
	destroyAllWindows();
	
}

void Fusion_processing::chromaCb(const sensor_msgs::ImageConstPtr& msg)
{
	Mat fusion;
	vector< Rect_<int> > fusion_rects;
	cv_bridge::CvImagePtr cv_ptr_dif;
	try
	{
		cv_ptr_dif 	 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	
	fusion 	 = (cv_ptr_dif->image);
	int height 	 = (msg->height);
	int width 	 = (msg->width);
	
	//Detect moving blobs
	detectBlobs(fusion, fusion_rects, 15, 1, false);
	
	//Track blobs
	track(fusion_rects, people, width, height, 3, 5*max_rank);
		
	//Calculate depth, position and features of tracked boxes
	if(depth_available)
	{
		for(int i = 0; i < people.tracked_boxes.size(); ++i)
		{
			Mat depth_rect = depth_Mat(people.tracked_boxes[i]);
			try
			{
				//Calculating depth 
				float depth = calculateDepth(depth_rect, people.tracked_pos[i]);
				
				//Calculating z_diff feature
				people.tracked_pos[i].z_diff = depth - people.tracked_pos[i].z;
				
				if(depth != 0)
					people.tracked_pos[i].z = depth;
				
				//Calculating Std of depth feature
				absdiff(depth_rect, people.tracked_pos[i].z, depth_rect);
				people.tracked_pos[i].depth_std = sum(depth_rect)[0]/(depth_rect.rows*depth_rect.cols); 
				
				
				//Visualize depth mat
				//~ Mat res;
				//~ depth = calculateDepth(depth_rect);
				//~ depthToGray(depth_rect, res, 0, max_depth);
				//~ imshow("depth", res);
				//~ moveWindow("depth", 645, 0);
			}
			catch(exception& e)
			{
				printf("%s %s", "Calculate depth failed: ", e.what());
			}
			
			try
			{
				//Calculate real world position, height, distance moved
				calculatePosition(people.tracked_boxes[i], people.tracked_pos[i]);
			}
			catch(exception& e)
			{
				printf("%s %s", "Calculate position failed: ", e.what());
			}
		}
		
		
	}
	
	
	if(display)
	{
		/*
		//For the box with the highest rank
		//Filter the image according to the estimated depth and visualize it
		Mat depth_rect = depth(rect);
		Mat depth_filtered(depth.rows, depth.cols, CV_8UC1);
		depth_filtered = Scalar(0);
		
		for(int i = 0; i < depth_rect.rows; i++)
		{
			float* cur = depth_rect.ptr<float>(i);
			for(int j = 0; j < depth_rect.cols; j++)
			{   
				if(abs(cur[j] - people.tracked_pos[index].z) > 300)
					cur[j] = 0;
			}   
		}
		depthToGray(depth_rect, depth_rect, min_depth, max_depth);
		
		depth_rect.copyTo(depth_filtered(people.tracked_boxes[index]));
		
		for(Rect rect: fusion_rects)
			rectangle(fusion, rect, 255, 1);
		*/
		for(int i = 0; i < people.tracked_boxes.size(); ++i)
		{
			rectangle(fusion, people.tracked_boxes[i], 255, 1);
		}
		imshow("fusion", fusion);
		moveWindow("fusion", 0, 0);
		
		waitKey(1);
	}
	
	
	ros::Time time = ros::Time::now();
	//Write csv file
	if(write_csv)
		writeCSV(people, session_path, time);

	//Publish results
	publishResults(people, time);
	previous_time = time;
}

void Fusion_processing::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	
	try
	{
		cv_ptr_depth    = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	depth_available = true;
	depth_Mat 		= (cv_ptr_depth->image);
}



/* Function that writes creates a csv file and appends values to it
 * 
 * PARAMETERS:
 *	    - collection: object that contains the bounded boxes detected
 *	    - path		: the root path of the csv file
 *	    - time		: ROS object that has the timestamp the frame was created
 * 
 * 
 * RETURN --
 */
void Fusion_processing::writeCSV(People& collection, string path, ros::Time time)
{		
	ofstream storage(path + "/fusion.csv" ,ios::out | ios::app );
	if(!collection.tracked_boxes.empty())
	{
		float time_interval  = (time - previous_time).toSec();
		for(int i = 0; i < collection.tracked_boxes.size(); ++i) 
		{
			float rank = collection.tracked_rankings[i];
			if(rank > 4)
			{
				Rect box = collection.tracked_boxes[i];
				Position pos = collection.tracked_pos[i];
				storage
					<<time<<"\t"
					<<i<<"\t"
					<<box.x<<"\t"
					<<box.y<<"\t"
					<<box.width<<"\t"
					<<box.height<<"\t"
					<<pos.ratio<<"\t"
					<<pos.ratio_diff/time_interval<<"\t"
					<<pos.distance/time_interval<<"\t"
					<<pos.distance_diff/time_interval<<"\t"
					<<pos.x_diff/time_interval<<"\t"
					<<pos.x_delta/time_interval<<"\t"
					<<pos.y_diff/time_interval<<"\t"
					<<pos.y_delta/time_interval<<"\t"
					<<pos.y_norm<<"\t"
					<<pos.y_norm_diff/time_interval<<"\t"
					<<pos.z_diff/time_interval<<"\t"
					<<abs(pos.z_diff)/time_interval<<"\t"
					<<pos.depth_std<<
				endl;
			}
			else
			{
				storage<<time<<endl;
			}
		}
	}
	else
	{
		storage<<time<<endl;
	}
	storage.close();
}

/* Creates a ROS message, populates it with the bounded boxes detected and their
 * metadata and publishes it  
 * 
 * PARAMETERS:
 *	    - collection: object that contains the bounded boxes detected
 *	    - time		: ROS object that has the timestamp the frame was created
 * 
 * 
 * RETURN --
 */
void Fusion_processing::publishResults(People& collection, ros::Time time){
	if (!collection.tracked_boxes.empty())
	{
		ros_visual_msgs::FusionMsg fmsg;

		fmsg.header.stamp = time;
		fmsg.header.frame_id = camera_frame;
		float time_interval  = (time - previous_time).toSec();
		for(int i = 0; i < collection.tracked_boxes.size() ; ++i) 
		{
			
			Rect box = collection.tracked_boxes[i];
			Position pos = collection.tracked_pos[i];
			
			ros_visual_msgs::Box box_;
			
			box_.id = i;
			box_.rect.x = box.x;
			box_.rect.y = box.y;
			box_.rect.width = box.width;
			box_.rect.height = box.height;
			box_.pos.ratio = pos.ratio;
			box_.pos.ratio_diff = pos.ratio_diff;
			box_.pos.distance = pos.distance;
			box_.pos.distance_diff = pos.distance_diff;
			box_.pos.x_diff = pos.x_diff/time_interval;
			box_.pos.x_delta = pos.x_delta/time_interval;
			box_.pos.y_diff = pos.y_diff/time_interval;
			box_.pos.y_delta = pos.y_delta/time_interval;
			box_.pos.y_norm = pos.y_norm;
			box_.pos.y_norm_diff = pos.y_norm_diff/time_interval;
			box_.pos.z_diff = pos.z_diff;
			box_.pos.z_diff_norm = pos.z_diff_norm;
			box_.pos.depth_std = pos.depth_std;
			
			
			fmsg.boxes.push_back(box_);
		}
		results_publisher.publish(fmsg);
	}
}

		
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  Fusion_processing fp;
  ros::spin();
  return 0;
}


		
	
