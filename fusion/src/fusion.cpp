#include <fusion.hpp>


Fusion_processing::Fusion_processing()
: it_(nh_)
{
	 //Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("results_topic", results_topic, string("results"));
	local_nh.param("image_topic", image_topic, string("/chroma_proc/image"));
	local_nh.param("image_dif_topic", image_dif_topic, string("/chroma_proc/image_dif"));
	local_nh.param("depth_topic", depth_topic, string("/depth_proc/image"));
	local_nh.param("project_path",path_, string(""));
	local_nh.param("playback_topics", playback_topics, false);
	local_nh.param("write_csv", write_csv, false);
	local_nh.param("display", display, false);
	local_nh.param("max_depth", max_depth, DEPTH_MAX);
	local_nh.param("min_depth", min_depth, DEPTH_MIN);
	local_nh.param("camera_frame", camera_frame, string("camera_link"));

	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Fusion_processing","Subscribing at compressed topics \n"); 
		image_topic += "/compressed";
		depth_topic += "/compressedDepth";
    } 
	
	
    
    ImageSubscriber *image_sub  = new ImageSubscriber(it_, image_topic, 3 );
    ImageSubscriber *image_dif_sub  = new ImageSubscriber(it_, image_dif_topic, 3 );
    ImageSubscriber *depth_sub  = new ImageSubscriber(it_, depth_topic, 3 );
	sync = new message_filters::Synchronizer< MySyncPolicy >( MySyncPolicy( 5 ), *image_sub, *image_dif_sub, *depth_sub );
    sync->registerCallback( boost::bind( &Fusion_processing::callback, this, _1, _2, _3) );
    
    results_publisher = local_nh.advertise<fusion::FusionMsg>(results_topic, 1000);

    if(write_csv)
    {
	    Utility u;
		session_path = u.initialize(path_, true, false);
	}
}

Fusion_processing::~Fusion_processing()
{
	//destroy GUI windows
	destroyAllWindows();
	
}

void Fusion_processing::callback(const sensor_msgs::ImageConstPtr& chroma_msg, const sensor_msgs::ImageConstPtr& chroma_dif_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
	Mat fusion;
	Mat chroma;
	Mat chroma_dif;
	Mat depth;
	vector< Rect_<int> > fusion_rects;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_dif;
	cv_bridge::CvImagePtr cv_ptr_depth;
	
	
	cv_ptr = cv_bridge::toCvCopy(chroma_msg, sensor_msgs::image_encodings::MONO8);	
	cv_ptr_dif = cv_bridge::toCvCopy(chroma_dif_msg, sensor_msgs::image_encodings::MONO8);	
	cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	
	chroma = (cv_ptr->image).clone();
	chroma_dif = (cv_ptr_dif->image).clone();
	depth = (cv_ptr_depth->image).clone();
	
	//Fuse the gray and depth images
	fusion = chroma_dif;
	cv::threshold(fusion, fusion, 100, 255, THRESH_BINARY);
	
	//Detect moving blobs
	detectBlobs(fusion, fusion_rects, 15);
	
	//Track blobs
	track(fusion_rects, people);
	
	//Check which tracked box has the highest rank
	//and draw the boxes for visualization
	Rect rect;
	int rank = -1;
	int index = -1;
	Position pos;
	int end = people.tracked_rankings.size();
	for(int i = 0; i < end; i++)
	{
		people.tracked_pos.push_back(pos);
		if(people.tracked_rankings[i] > 3)
		{
			//~ cout<<people.tracked_boxes[i].x<<" " <<people.tracked_boxes[i].y<<" "<<people.tracked_boxes[i].width<<" "<<people.tracked_boxes[i].height<<endl;
			people.tracked_pos.push_back(pos);
			rectangle(fusion, people.tracked_boxes[i], 255, 1);
			rectangle(chroma, people.tracked_boxes[i], 0, 1);
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
	
	
	//For the box with the highest rank
	if(rect.width > 0)
	{
	
		Mat depth_rect = depth(rect);
		
		try
		{
			//Estimate its depth
			people.tracked_pos[index].z = calculateDepth(depth_rect, people.tracked_boxes[index]);
		}
		catch(exception& e)
		{
			printf("%s %s", "Calculate depth failed: ", e.what());
		}
		
		//Filter the image according to the estimated depth and visualize it
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
		
		depth_rect.copyTo(depth_filtered(rect));
		
		if(display)
		{
			imshow("fusion", fusion);
			moveWindow("fusion", 0, 0);
			imshow("depth_filt", depth_filtered);
			moveWindow("depth_filt", 645, 550);
			imshow("chroma", chroma);
			moveWindow("chroma", 0, 550);
			waitKey(1);
		}
		
		try
		{
			//Calculate real world position and height, distance moved
			calculatePosition(people.tracked_boxes[index], people.tracked_pos[index]);
		}
		catch(exception& e)
		{
			printf("%s %s", "Calculate position failed: ", e.what());
		}
		
	
		//~ cout<<"X:  "<<people.tracked_pos[index].x<<endl;
		//~ cout<<"Y:  "<<people.tracked_pos[index].y<<endl;
		//~ cout<<"Depth:  "<<people.tracked_pos[index].z<<endl;
		//~ cout<<"Height:  "<<people.tracked_pos[index].height<<endl;
		//~ cout<<"Distance:  "<<people.tracked_pos[index].distance<<endl;
		//~ cout<<endl;
		
		if(write_csv)
		{

			ros::Time time = cv_ptr->header.stamp;
			writeCSV(people, session_path, time);
		}		

		publishResults(people);

		
	}
	
}

void Fusion_processing::writeCSV(People& collection, string path, ros::Time time)
{		
	if (!collection.tracked_boxes.empty())
	{
		ofstream storage;
		
		char const *pchar = (path + "/csv/session.csv").c_str();  
		storage.open (pchar,ios::out | ios::app );
		
		
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
		storage.close();
	}
}

void Fusion_processing::publishResults(People& collection){
	if (!collection.tracked_boxes.empty())
	{
		fusion::FusionMsg fmsg;

		fmsg.header.stamp = ros::Time::now();
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


		
	
