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

	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Fusion_processing","Subscribing at compressed topics \n"); 
		image_topic += "/compressed";
		depth_topic += "/compressedDepth";
    } 
	
    image_sub = it_.subscribe(image_dif_topic, 10, &Fusion_processing::chromaCb, this);
    depth_sub = it_.subscribe(depth_topic, 10, &Fusion_processing::depthCb, this);
    
    results_publisher = local_nh.advertise<fusion::FusionMsg>(results_topic, 100);
	
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
	
	cv_ptr_dif 	 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	
	fusion 	 = (cv_ptr_dif->image).clone();
	cv::threshold(fusion, fusion, 100, 255, THRESH_BINARY);
	
	//Detect moving blobs
	detectBlobs(fusion, fusion_rects, 15, true);
	
	//Track blobs
	track(fusion_rects, people);
	
	//Check which tracked box has the highest rank
	//and draw the boxes for visualization
	Rect rect;
	Position pos;
	int rank  = -1;
	int index = -1;
	int end	  = people.tracked_rankings.size();
	for(int i = 0; i < end; i++)
	{
		people.tracked_pos.push_back(pos);
		if(people.tracked_rankings[i] > 3)
		{
			//~ cout<<people.tracked_boxes[i].x<<" " <<people.tracked_boxes[i].y<<" "<<people.tracked_boxes[i].width<<" "<<people.tracked_boxes[i].height<<endl;
			people.tracked_pos.push_back(pos);
			rectangle(fusion, people.tracked_boxes[i], 255, 1);
			//~ rectangle(chroma, people.tracked_boxes[i], 0, 1);
			if(rank < people.tracked_rankings[i])
			{
				rank = people.tracked_rankings[i];
				rect = people.tracked_boxes[i];
				index = i;
			}
		}
		else
		{
			people.tracked_boxes[i] = people.tracked_boxes.back();
			people.tracked_boxes.pop_back();
			people.tracked_rankings[i] = people.tracked_rankings.back();
			people.tracked_rankings.pop_back();
			i--;
			end--;
		}
	}
	
	end	  = people.tracked_boxes.size();
	for(int i = 0; i < end; i++)
	{
		if(rect.width > 0 && depth_available)
		{
			Mat depth_rect = depth(people.tracked_boxes[i]);
			try
			{
				//Estimate its depth
				people.tracked_pos[i].z = calculateDepth(depth_rect, people.tracked_boxes[i]);
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
		//~ //For the box with the highest rank
		//~ //Filter the image according to the estimated depth and visualize it
		//~ Mat depth_rect = depth(rect);
		//~ Mat depth_filtered(depth.rows, depth.cols, CV_8UC1);
		//~ depth_filtered = Scalar(0);
		
		//~ for(int i = 0; i < depth_rect.rows; i++)
		//~ {
			//~ float* cur = depth_rect.ptr<float>(i);
			//~ for(int j = 0; j < depth_rect.cols; j++)
			//~ {   
				//~ if(abs(cur[j] - people.tracked_pos[index].z) > 300)
					//~ cur[j] = 0;
			//~ }   
		//~ }
		//~ depthToGray(depth_rect, depth_rect, min_depth, max_depth);
		
		//~ depth_rect.copyTo(depth_filtered(people.tracked_boxes[index]));
		
		//~ imshow("fusion", fusion);
		//~ moveWindow("fusion", 0, 0);
		//~ imshow("depth_filt", depth_filtered);
		//~ moveWindow("depth_filt", 645, 550);
		//~ imshow("chroma", chroma);
		//~ moveWindow("chroma", 0, 550);
		//~ waitKey(1);
	}
	//~ cout<<"X:  "<<people.tracked_pos[index].x<<endl;
	//~ cout<<"Y:  "<<people.tracked_pos[index].y<<endl;
	//~ cout<<"Depth:  "<<people.tracked_pos[index].z<<endl;
	//~ cout<<"Height:  "<<people.tracked_pos[index].height<<endl;
	//~ cout<<"Distance:  "<<people.tracked_pos[index].distance<<endl;
	//~ cout<<endl;
	
	
	ros::Time time = cv_ptr_dif->header.stamp;
	
	//Write csv file
	if(write_csv)
		writeCSV(people, session_path, time);

	//Publish results
	publishResults(people, time);

	
}

void Fusion_processing::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr_depth;
	depth_available = true;
	cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	depth 		 = (cv_ptr_depth->image).clone();
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
	if (!collection.tracked_boxes.empty())
	{

		for(int i = 0; i < collection.tracked_boxes.size() ; i++) 
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
				<<pos.x<<"\t"
				<<pos.y<<"\t"
				<<pos.z<<"\t"
				<<pos.top<<"\t"
				<<pos.height<<"\t"
				<<pos.distance<<
			endl;
		}
	}
	else
	{
		storage
			<<time<<"\t"<<
		endl;
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
		fusion::FusionMsg fmsg;

		fmsg.header.stamp = time;
		fmsg.header.frame_id = camera_frame;
		
		for(int i = 0; i < collection.tracked_boxes.size() ; i++) 
		{
			Rect box = collection.tracked_boxes[i];
			Position pos = collection.tracked_pos[i];
			
			fusion::Box box_;
			
			box_.id = i;
			box_.rect.x = box.x;
			box_.rect.y = box.y;
			box_.rect.width = box.width;
			box_.rect.height = box.height;
			box_.pos.x = pos.x;
			box_.pos.y = pos.y;
			box_.pos.z = pos.z;
			box_.pos.top = pos.top;
			box_.pos.height = pos.height;
			box_.pos.distance = pos.distance;
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


		
	
