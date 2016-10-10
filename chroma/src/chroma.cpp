#include <chroma.hpp>

Chroma_processing::Chroma_processing()
: it_(nh_)
{
	//Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("image_topic"		 , image_topic		   ,string("/camera/rgb/image_raw"));
	local_nh.param("image_out_topic"	 , image_out_topic	   , string("/chroma_proc/image"));
	local_nh.param("image_out_dif_topic" , image_out_dif_topic , string("/chroma_proc/image_dif"));
	local_nh.param("project_path"		 , path_  			   , string(""));
	local_nh.param("playback_topics"	 , playback_topics	   , false);
	local_nh.param("display"			 , display	 		   , false);
	
	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Chroma_processing","Subscribing at compressed topics \n"); 
		
		image_sub = it_.subscribe(image_topic, 10, 
		  &Chroma_processing::imageCb, this, image_transport::TransportHints("compressed"));
    }
    else
		image_sub = it_.subscribe(image_topic, 10, &Chroma_processing::imageCb, this);
		
	
	image_pub 	  = it_.advertise(image_out_topic, 100); 
	image_pub_dif = it_.advertise(image_out_dif_topic, 100); 
}

Chroma_processing::~Chroma_processing()
{
	//destroy GUI windows
	destroyAllWindows();
	
}

/* Callback function to handle ROS image messages
 * 
 * PARAMETERS:
 * 			- msg : ROS message that contains the image and its metadata
 * 
 * RETURN: --
 */
void Chroma_processing::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	
	int rows;
	int cols;
	int channels;
	int size;
	cv_bridge::CvImagePtr cv_ptr;
	
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);	  
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	cur_rgb = (cv_ptr->image).clone();
	
	// contrast fix
	cur_rgb.convertTo(cur_rgb, -1, 2, 0);
	
	// gamma correction
	gammaCorrection(cur_rgb);
	

	// First run variable initialization 
	if(frameCounter == -1 )
	{
		rows 	 = cur_rgb.rows;
		cols 	 = cur_rgb.cols;
		channels = cur_rgb.channels();
		size 	 = rows*cols*channels;
		ref_rgb  = cur_rgb.clone();
		
		frameCounter++;
	}
	
	//Calculating image difference between the current and previous images
	frameDif(cur_rgb, ref_rgb, dif_rgb, 255*0.33);
	ref_rgb = cur_rgb.clone();

	
	//Display
	 
	//Blob detection
	if(display)
	{
		detectBlobs(dif_rgb, rgb_rects, 15, true);
		
		Mat temp = dif_rgb.clone();
	    for(Rect rect: rgb_rects)
			rectangle(temp, rect, 255, 1);
		rgb_rects.clear();
		
		imshow("dif_rgb", temp);
		moveWindow("dif_rgb", 0, 0);
		imshow("cur_rgb", cur_rgb);
		moveWindow("cur_rgb", 645, 0);
		waitKey(1);
	}
	
	///////////////////////////////
	///////////////////////////////

	/*  Image background estimation	
	 * 
	 * 
	cur_rgb = (cv_ptr->image).clone();
	
	// Edge detection and background estimation
	int kernel_size = 3;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	
	
	cur_rgb.convertTo(cur_rgb, -1, 2, 0);
    gammaCorrection(cur_rgb);
	//~ GaussianBlur( cur_rgb, cur_rgb, Size(3,3), 0, 0, BORDER_DEFAULT );

	medianBlur(cur_rgb, cur_rgb, 3);
	Laplacian( cur_rgb, cur_rgb, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( cur_rgb, cur_rgb );
	

    estimateBackground(cur_rgb, back_Mat, rgb_storage, 500, 0.04);
    Mat temp_Mat = cur_rgb.clone();
    estimateForeground(cur_rgb, back_Mat, temp_Mat);
    medianBlur(temp_Mat, temp_Mat, 7);
    adaptiveThreshold(temp_Mat, temp_Mat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, -15);
    
    vector< Rect_<int> > back_rects;
    detectBlobs(temp_Mat, back_rects, 15);
	
    imshow("temp_Mat", temp_Mat);
	moveWindow("temp_Mat", 645, 0);
	*/
	
	has_image = true;
	
	//Publish processed image
	cv_ptr->image = cur_rgb;
	image_pub.publish(cv_ptr->toImageMsg());
	
	//Publish image difference
	cv_ptr->image = dif_rgb;
	image_pub_dif.publish(cv_ptr->toImageMsg());
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "chroma");
	Chroma_processing ip;
	ros::spin();	
	return 0;
}
