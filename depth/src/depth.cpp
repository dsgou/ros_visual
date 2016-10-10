#include <depth.hpp>

	
Depth_processing::Depth_processing()
: it_(nh_)
{
    //Getting the parameters specified by the launch file 
    ros::NodeHandle local_nh("~");
    local_nh.param("depth_topic"		, depth_topic		, string("/camera/depth/image_raw"));
    local_nh.param("depth_out_image_topic"  , depth_out_image_topic , string("/depth_proc/image"));
    local_nh.param("project_path"		,path_			, string(""));
    local_nh.param("playback_topics"	, playback_topics	,false);
    local_nh.param("display"		, display		, false);
    local_nh.param("max_depth"		, max_depth		, DEPTH_MAX);
    local_nh.param("min_depth"		, min_depth		, DEPTH_MIN);
    
    if(playback_topics)
    {
	ROS_INFO_STREAM_NAMED("Depth_processing","Subscribing at compressed topics \n"); 
			
	depth_sub = it_.subscribe(depth_topic, 10, 
	   &Depth_processing::depthCb, this, image_transport::TransportHints("compressedDepth"));
    }
    else	  
	depth_sub = it_.subscribe(depth_topic, 10, &Depth_processing::depthCb, this);
	
    depth_pub = it_.advertise(depth_out_image_topic, 100);
}

Depth_processing::~Depth_processing()
{
	//destroy GUI windows
	destroyAllWindows();
	
}


/* Callback function to handle ROS depth messages
 * 
 * PARAMETERS:
 *	    - msg: ROS message that contains the depth image its metadata
 * 
 * 
 * RETURN --
 */
void Depth_processing::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
    Mat cur_depth;
    Mat temp_depth;
    int morph_elem = 0;
    int morph_size = 2;
    int morph_operator = 0;
    cv_bridge::CvImagePtr cv_ptr_depth;
    
    try
    {
	    cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)	
    {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
    }
    
    cur_depth = (cv_ptr_depth->image);
    //Converting depth values to 0-255
    depthToGray(cur_depth, cur_depth, min_depth, max_depth);
    
    if (dFrameCounter == -1)
    {
	    ref_depth = cur_depth.clone();
	    dFrameCounter++;
    }
    
    //Depth preprocessing
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    
    Mat depth_dif;
    morphologyEx(cur_depth, cur_depth, 3, element);
    
    //Filling areas in the depth image that have no value because
    //of sensor noise or surface reflectivity
    rectFill(cur_depth, 0.3, 2);
    upVerticalFill(cur_depth, 0.3, true);
    

    /* Frame difference
     * 
     * 
    temp_depth = cur_depth.clone();

    medianBlur(temp_depth, temp_depth, 15);
    
    frameDif(temp_depth, ref_depth, dif_depth, 255*0.07);
    
    morphologyEx(dif_depth, dif_depth, 2, element);
    
    ref_depth = temp_depth.clone();
    */
    
    
    //Display
    if(display)
    {
	    
	    imshow("cur_depth", cur_depth);
	    moveWindow("cur_depth", 0, 0);
	    
	/*
	 * 
	    //blob detection
	detectBlobs(dif_depth, depth_rects, 15);
	
	//display
	Mat temp = dif_depth.clone();
	for(Rect rect: depth_rects)
		rectangle(temp, rect, 255, 1);
	depth_rects.clear();
	
	
	imshow("dif_depth", temp);
	moveWindow("dif_depth", 0, 550);
	*/
    }

    /* Depth backgound estimation 
    * 
    * 
    //~ int kernel_size = 3;
    //~ int scale = ;
    //~ int delta = 0;
    //~ int ddepth = CV_16S;
    //~ 
    //~ Laplacian( cur_depth, cur_depth, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
    //~ 
    //~ convertScaleAbs( cur_depth, cur_depth );
    Mat detected_edges;
    Mat  whatever;
    whatever.create( cur_depth.size(), cur_depth.type() );
    int lowThreshold = 500;
    int ratio = 3;
    int kernel_size = 5;
    
    
    blur( cur_depth, cur_depth, Size(3,3) );
    Canny( cur_depth, cur_depth, lowThreshold, lowThreshold*ratio, kernel_size );
    whatever = Scalar::all(0);

    //~ cur_depth.copyTo( whatever, detected_edges);
    
    imshow("whatever", cur_depth);
    moveWindow("whatever", 0, 550);

    
    Mat back_depth;
estimateBackground(cur_depth, back_depth, depth_storage, 100, 0.025);

Mat back_dif = cur_depth.clone();
estimateForeground(cur_depth, back_depth, back_dif);
    
//~ adaptiveThreshold(back_dif, back_dif, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, -5);
//~ medianBlur(back_dif, back_dif, 3);

vector< Rect_<int> > back_dif_rects;
detectBlobs(back_dif, back_dif_rects, 3);

    imshow("back_dif", back_dif);
    moveWindow("back_dif", 645, 550);
    */
    
    waitKey(1);
    
    //Converting chroma values to meters
    grayToDepth(cur_depth, cur_depth, max_depth);
    
    
    //Publish corrected depth image
    cv_ptr_depth->image = cur_depth;
    depth_pub.publish(cv_ptr_depth->toImageMsg());
	

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "depth");
	Depth_processing ip;
	ros::spin();	
	return 0;
}
