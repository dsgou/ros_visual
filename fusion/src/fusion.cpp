#include <fusion.hpp>


Fusion_processing::Fusion_processing()
: it_(nh_)
{
	 //Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("image_topic", image_topic, string("/chroma_proc/image_raw"));
	local_nh.param("depth_topic", depth_topic, string("/depth_proc/image"));
	local_nh.param("depth_dif_topic", depth_dif_topic, string("/depth_proc/image_dif"));
	local_nh.param("project_path",path_, string(""));
	local_nh.param("playback_topics", playback_topics, false);
	local_nh.param("write_csv", write_csv, false);
	local_nh.param("display", display, false);
	local_nh.param("max_depth", max_depth, DEPTH_MAX);
	local_nh.param("min_depth", min_depth, DEPTH_MIN);
	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Fusion_processing","Subscribing at compressed topics \n"); 
		image_topic += "/compressed";
		depth_topic += "/compressedDepth";
    } 
	
	
    
    ImageSubscriber *image_sub  = new ImageSubscriber(it_, image_topic, 3 );
    ImageSubscriber *depth_sub  = new ImageSubscriber(it_, depth_topic, 3 );
    ImageSubscriber *depth_dif_sub  = new ImageSubscriber(it_, depth_dif_topic, 3 );
	sync = new message_filters::Synchronizer< MySyncPolicy >( MySyncPolicy( 5 ), *image_sub, *depth_sub, *depth_dif_sub );
    sync->registerCallback( boost::bind( &Fusion_processing::callback, this, _1, _2, _3 ) );
    
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

void Fusion_processing::callback(const sensor_msgs::ImageConstPtr& chroma_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& depth_dif_msg)
{
	Mat fusion;
	Mat chroma;
	Mat depth;
	Mat depth_dif;
	vector< Rect_<int> > fusion_rects;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;
	cv_bridge::CvImagePtr cv_ptr_depth_dif;
	
	
	cv_ptr = cv_bridge::toCvCopy(chroma_msg, sensor_msgs::image_encodings::MONO8);	
	cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	cv_ptr_depth_dif = cv_bridge::toCvCopy(depth_dif_msg, sensor_msgs::image_encodings::MONO8);
	
	chroma = (cv_ptr->image).clone();
	depth = (cv_ptr_depth->image).clone();
	depth_dif = (cv_ptr_depth_dif->image).clone();
	imshow("chroma", chroma);
	moveWindow("chroma", 0, 0);
	imshow("depth", depth);
	moveWindow("depth", 645, 0);
	imshow("depth_dif", depth_dif);
	moveWindow("depth_dif", 0, 550);
	
	//Fuse the gray and depth images
	fusion = chroma/2 +  depth_dif/2;
	normalize(fusion, fusion, 0, 255, NORM_MINMAX);
	cv::threshold(fusion, fusion, 110, 255, THRESH_BINARY);
	
	//Detect moving blobs
	detectBlobs(fusion, fusion_rects, 10);
	
	//Track blobs
	track(fusion_rects, people);
	
	//Check which tracked box has the highest rank
	//and draw the boxes for visualization
	Rect rect;
	int rank = -1;
	int index = -1;
	Fusion_processing::Position pos;
	for(int i = 0; i < people.tracked_rankings.size(); i++)
	{
		people.tracked_pos.push_back(pos);
		if(people.tracked_rankings[i] > 3)
		{
			//~ cout<<people.tracked_boxes[i].x<<" " <<people.tracked_boxes[i].y<<" "<<people.tracked_boxes[i].width<<" "<<people.tracked_boxes[i].height<<endl;
			rectangle(fusion, people.tracked_boxes[i], 255, 1);
			rectangle(chroma, people.tracked_boxes[i], 0, 1);
			if(rank < people.tracked_rankings[i])
			{
				rank = people.tracked_rankings[i];
				rect = people.tracked_boxes[i];
				index = i;
			}
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
		
		depth_dif = Scalar(0);
		
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
		
		depth_rect.copyTo(depth_dif(rect));
		imshow("depth_filtered", depth_dif);
		moveWindow("depth_filtered", 645, 550);
		
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
			writeCSV(people, session_path);
		}		
		
	}
	
	waitKey(1);
	
}

void Fusion_processing::writeCSV(Fusion_processing::People& collection, string path)
{		
	if (!collection.tracked_boxes.empty())
	{
		ofstream storage;
		
		char const *pchar = (path + "/csv/session.csv").c_str();  
		storage.open (pchar,ios::out | ios::app );
		
		gettimeofday(&tv, NULL); 
		curtime=tv.tv_sec;
		strftime(timeBuf,sizeof(timeBuf),"%T:",localtime(&curtime));
		
		for(int i = 0; i < collection.tracked_boxes.size() ; i++) 
		{
			Rect box = collection.tracked_boxes[i];
			Fusion_processing::Position pos = collection.tracked_pos[i];
			storage
				<<timeBuf<<int(tv.tv_usec/10000)<<","
				<<i<<"\t"
				<<box.x<<"\t"
				<<box.y<<"\t"
				<<box.width<<"\t"
				<<box.height<<"\t"
				<<pos.x<<"\t"
				<<pos.y<<"\t"
				<<pos.z<<"\t"
				<<pos.height<<"\t"
				<<pos.distance<<
				endl;
				
		}
		
		storage.close();
	}
}

/* Detects non-black areas in the image and populates a list of OpenCV Rects  
 *
 * PARAMETERS: -the vector that will be populated
 * 			   -screen width
 * 
 * RETURN --
 */
void Fusion_processing::detectBlobs(Mat& src, vector< Rect_<int> >& colour_areas, int range)
{
	float detectionFactor = 0.1;
	float mergeFactor = 0.1;
	bool flag = false;
	int channels = src.channels();
	int cols = src.cols;
	int rows = src.rows;
	int size = cols*rows*channels;
	for(int y = 0; y < 1; y++)
	{
		const uchar *dif = src.ptr<uchar>(y);
		for(int x = 0; x < size; x = x + channels)
		{
			if(dif[x] != 0)
			{
				int i ,j, o;
				i = (x/channels)%(cols);
				j = floor(x/(cols*channels));
				
				Rect_<int> removal;
				if(i + range >= cols)
					continue;
				if(j + range >= rows)
					continue;
				
				removal = Rect(i, j, range , range);
				if(removal.width < 1 || removal.height < 1)
					continue;
					
				if(!colour_areas.empty())
				{
					for(int k = 0; k < colour_areas.size(); k++)
					{
						Rect_<int> rect = colour_areas[k];
						Rect all = removal | rect;
						int temp = removal.y;
						removal.y = rect.y;
						Rect intersection = removal & rect;
						removal.y = temp;
						int threshold = intersection.area();
						if(threshold < 1)
							continue;
							
						if(removal.area() < rect.area())
						{
							if(threshold > detectionFactor*removal.area())
							{
								colour_areas[k] = all;
								flag = true;
							}
						}
						else
						{
							if(threshold > detectionFactor*rect.area())
							{
								colour_areas[k] = all;
								flag = true;
							}
						}
					}
					if(!flag)
					{
						colour_areas.push_back(removal);
					}
					else
						flag = false;
						
					int end = colour_areas.size();
					for(int a = 0; a < end; a++) 
					{
						for(int b = a + 1; b < end; b++) 
						{	
							Rect_<int> removal = colour_areas[a];
							Rect_<int> rect = colour_areas[b];
							
							Rect all = removal | rect;
							int temp = removal.y;
							removal.y = rect.y;
							Rect intersection = removal & rect;
							removal.y = temp;
							int threshold = intersection.area();
							if(threshold < 1)
								continue;
								
							if(removal.area() < rect.area())
							{
								if(threshold > mergeFactor*removal.area())
								{
									colour_areas[a] = all;
									colour_areas[b] = colour_areas.back();
									colour_areas.pop_back();
									b = a + 1;
									end--;
								}
							}
							else
							{
								if(threshold > mergeFactor*rect.area())
								{
									colour_areas[a] = all;
									colour_areas[b] = colour_areas.back();
									colour_areas.pop_back();
									b = a + 1;
									end--;
								}
							}
							
						}
					}
					
				
				}
				else
				{
					colour_areas.push_back(removal);
				}
				
			}
		}
	}
	int end = colour_areas.size();
	
	for(int k = 0; k < end; k++)
	{
		float width  = colour_areas[k].width;
		float height = colour_areas[k].height;
		float x  = colour_areas[k].x;
		float y = colour_areas[k].y;
		if((x < 0) || (y < 0) || (height < 0) || (width < 0))
		{
			//~ rectangle(src, colour_areas[k], 0, CV_FILLED);
			cout<<"done"<<endl;
			colour_areas[k] = colour_areas.back();
			colour_areas.pop_back();
			k = k <0? 0: k--;
			end = end <0? 0: end--;
				
		}
	}
	
	end = colour_areas.size();
	for(int k = 0; k < end; k++)
	{
		float width  = colour_areas[k].width;
		float height = colour_areas[k].height;
		float area = colour_areas[k].area();
		float ratio = width/height;
		if((ratio < 0.25) || (ratio > 1.5) || (area < cols*rows*0.02))
		{
			//~ rectangle(src, colour_areas[k], 0, CV_FILLED);
			colour_areas[k] = colour_areas.back();
			colour_areas.pop_back();
			k--;
			end--;
				
		}
	}
	
	/*
	for(int k = 0; k < end; k++)
	{
		if(colour_areas[k].width > 250)
		{
			colour_areas[k].width = colour_areas[k].width/2;
			Rect_<int> rect = colour_areas[k];
			rect.x = rect.x + rect.width;
			rect.width = colour_areas[k].width;
			colour_areas.push_back(rect);
			k--;
			end++;	
			
		}
			
	}
	*/
	
	//~ cout<<"Size: "<<colour_areas.size()<<endl;
	//~ for(Rect rect: colour_areas)
		//~ cout<<rect.x<<" "<<rect.y<<" "<<rect.width<<" "<<rect.height<<endl;
		//~ rectangle(src, rect, CV_RGB(255, 255, 255), 1);
}

/* Tracks current rectangle in the image and populates a
 * collection. Every tracked box has a rank(=3) that increases if the box
 * is redetected and decreases otherwise. The threshold that is used to compare
 * the detected boxes with the stored ones. 
 *
 * PARAMETERS: -current image rectangles
 * 			   -the collection to be populated
 * 			   -the initial rank of a new box, 
 * 			   -rectangle comparison threshold
 * 
 * RETURN --
 */
void Fusion_processing::track(vector< Rect_<int> >& cur_boxes, People& collection, int rank, float threshold)
{
	bool exists = false;
	float step = 1.2;
	
	
	if(!cur_boxes.empty())
	{	
		
		for(int a = 0; a < cur_boxes.size(); a++) 
		{
			exists = false;
			for(int b = 0; b < collection.tracked_boxes.size(); b++) 
			{
				Rect intersection = cur_boxes[a] & collection.tracked_boxes[b];
				int area = intersection.area();	
				if(cur_boxes[a].area() < collection.tracked_boxes[b].area())
				{
					if(area > threshold*cur_boxes[a].area())
					{
						collection.tracked_boxes[b] = cur_boxes[a];
						if(collection.tracked_rankings[b] <= 10)
							collection.tracked_rankings[b] = collection.tracked_rankings[b] + step;
						exists = true;	
						break;	
					}
				}
				else
				{
					if(area > threshold*collection.tracked_boxes[b].area())
					{
						collection.tracked_boxes[b] = cur_boxes[a];
						if(collection.tracked_rankings[b] <= 10)
							collection.tracked_rankings[b] = collection.tracked_rankings[b] + step;
						exists = true;	
						break;	
					}
				}
				
			}
			if(!exists)
			{
				collection.tracked_boxes.push_back(cur_boxes[a]);
				collection.tracked_rankings.push_back(rank + step);
			}
		}
		for(int a = 0; a < collection.tracked_boxes.size(); a++)
		{
			collection.tracked_rankings[a] = collection.tracked_rankings[a] - 1;
		}
		for(int a = 0; a < collection.tracked_boxes.size(); a++)
		{
			if(collection.tracked_rankings[a] <= 0 && collection.tracked_boxes.size() > 0)
			{
				collection.tracked_boxes[a] = collection.tracked_boxes.back();
				collection.tracked_boxes.pop_back();
				collection.tracked_rankings[a] = collection.tracked_rankings.back();
				collection.tracked_rankings.pop_back();
				a--;
			}
			
		}
		
	}
}

/*Converts a grayscale Mat to a depth Mat by using the maximum depth value
 * 
 * PARAMETERS:
 * 			-Mat depth
 * 			-Mat with the graysclae result
 * 			-float minimum depth
 * 			-float maximum depth
 * 
 * RETURN: --
 */
void Fusion_processing::grayToDepth(Mat& src, Mat& dst, float max_depth)
{
	Mat temp_img(src.rows, src.cols, CV_32FC1);
	int cols = src.cols;
	int rows = src.rows;
	if(src.isContinuous())
	{
	    cols *= rows;
	    rows = 1;
	}
	for(int i = 0; i < rows; i++)
	{
		uchar* cur = src.ptr<uchar>(i);
		float* Ii = temp_img.ptr<float>(i);
		for(int j = 0; j < cols; j++)
		{   
			Ii[j] = (max_depth*(float(cur[j])/(255.0)));
		}   
	}
	dst = temp_img.clone();
	
}

/*Converts a depth Mat to a grayscale one by using the min and maximum depth values
 * 
 * PARAMETERS:
 * 			-Mat depth
 * 			-Mat with the graysclae result
 * 			-float minimum depth
 * 			-float maximum depth
 * 
 * RETURN: --
 */
void Fusion_processing::depthToGray(Mat& src, Mat& dst, float min_depth, float max_depth)
{
	
	Mat temp_img(src.rows, src.cols, CV_8UC1);
	int cols = src.cols;
	int rows = src.rows;
	if(src.isContinuous())
	{
	    cols *= rows;
	    rows = 1;
	}
	for(int i = 0; i < rows; i++)
	{
		float* cur = src.ptr<float>(i);
		uchar* Ii = temp_img.ptr<uchar>(i);
		for(int j = 0; j < cols; j++)
		{   
			Ii[j] = (255*((cur[j] - min_depth)/(max_depth - min_depth)));
		}   
	}
	dst = temp_img.clone();
	
}

/* Calculates and stores the real world coordinates (x, y, z) of a face
 * in respect to the center of the camera.
 * 
 * PARAMETERS:
 * 			-Database object holding the found faces in the image
 * 
 * RETURN: --
 * 
 */
void Fusion_processing::calculatePosition(Rect_<int>& rect, Fusion_processing::Position& pos)
{
	
	float hor_x = 0.0;
	float hor_y = 0.0;
	float ver_x = 0.0;
	float ver_y = 0.0;
	float hor_focal = 0.0;
	float ver_focal = 0.0;
	float depth = 0.0;
	float distance = 0.0;
	float top = 0.0;
	float bottom = 0.0;
	
	
	depth = pos.z;
	if (depth != 0.0)
	{
		
		//Find the focal length 
		hor_focal = depth_height / (2 * tan((Vfield/2) * M_PI / 180.0) );
		ver_focal = depth_width / (2 * tan((Hfield/2) * M_PI / 180.0) );
		
		//Transform the pixel x, y in respect to 
		//the camera center
		ver_x = rect.x + rect.width/2;
		if(ver_x > depth_width/2)
			ver_x = (ver_x - depth_width/2);
		else
			ver_x = (-1)*(depth_width/2 - ver_x);
		
		ver_y = rect.y + rect.height/2;
		if(ver_y > depth_height/2)
			ver_y = (-1)*(ver_y - depth_height/2);
		else
			ver_y = (depth_height/2 - ver_y);
		
		//Calculate the real world coordinates of the box center
		hor_y = depth * ver_y / hor_focal;
		hor_x = depth * ver_x / ver_focal;
		
		if(pos.x != 0)
		{
			distance = abs(pos.x - hor_x);
			pos.distance = distance;
		}
		pos.x = hor_x;
		pos.y = hor_y;
		
		
		
		
		ver_y = rect.y;
		if(ver_y > depth_height/2)
			ver_y = (-1)*(ver_y - depth_height/2);
		else
			ver_y = (depth_height/2 - ver_y);
		top = depth * ver_y / hor_focal;
		
		
		ver_y = rect.y + rect.height;
		if(ver_y > depth_height/2)
			ver_y = (-1)*(ver_y - depth_height/2);
		else
			ver_y = (depth_height/2 - ver_y);
			
		bottom = depth * ver_y / hor_focal;
		pos.height = abs(top - bottom);
	
	}
	
}

/* Calculates the depth of the closest object in the specified Mat by using clustering
 * 
 * PARAMETERS:
 * 		-Mat
 * 		-number of values to calculate
 * 		
 * RETURN:   
 * 		-Double holding the min cluster median value
 * 
 */
double Fusion_processing::calculateDepth(Mat& src, Rect_<int> personRect)
{
	
	int clusters = 2;
	int cols = src.cols;
	int rows = src.rows;
	int channels = src.channels();
	int size = cols*rows*channels;
	double median = 0.0;
	double saveMin = 0.0;
	double saveCluster = 0.0;
	double saveCenter = 0.0;
	double depthCombined = 0.0;
	vector<double> vec;
	vector<float> cluster_vec;
	
	if(cols > 0 && rows > 0)
	{
		
		if(src.isContinuous())
		{
			rows = 1;
			cols = size;
		}
		
		cols = src.cols*0.8;
		rows = src.rows*0.8;
		
		for(int i = 0; i < rows; i++)
		{
			const float* cur = src.ptr<float>(i);
			for(int j = 0; j < cols; j++)
			{
				cluster_vec.push_back(cur[j]);
			}
			
		}
		
		// clustering 
		kmeansreport rep = clusterize(cluster_vec, clusters);
		
		// cluster item count
		int counter[clusters];
		for(int i = 0; i < clusters; i++)
		{
			counter[i] = 0;
		}
		for(int i = 0; i < rep.cidx.length(); i++)
		{
			if(rep.cidx[i] > 0 && rep.cidx[i] < clusters)
				counter[rep.cidx[i]]++;			
		}
		
		//check which centroid has the smallest value
		int min_centroid = INT_MAX;
		int index [clusters + 1];
		for(int i = 0; i < clusters + 1; i++)
		{
			index[i] = -1;
		}
		
		if(rep.c.rows() > 0)
		{	
			for(int i = 0; i < clusters; i++)
			{
				if((min_centroid > rep.c[i][0]) && (rep.c[i][0] > 0))
				{
					min_centroid = rep.c[i][0];
					index[0] = i;
				}
			}
			index[index[0] + 1] = INT_MAX;
			
			//~ cout<<rep.c[0][0] <<" "<< rep.c[1][0]<<" "<<min_centroid<< " " <<index[0]<<endl;
			//~ cout<<counter[0] <<" "<< counter[1]<<" "<<endl;
			
			
			//populate vector with the points of the cluster
			//with the smallest centroid
			int loop = 0; 
			while((median == 0) && (loop < clusters))
			{
				loop++;
				cluster_vec.clear();
				for(int i = 0; i < rows; i++)
				{
					const float* cur = src.ptr<float>(i);	
					for(int j = 0; j < cols; j++)
					{
						if(index[0] == rep.cidx[i*cols + j])
						{
							cluster_vec.push_back(cur[j]);
						}
					}
				}
				
				//median
				nth_element(cluster_vec.begin(), cluster_vec.begin() + cluster_vec.size()/2, cluster_vec.end());
				median = cluster_vec[cluster_vec.size()/2];
				for(int i = 1; i < clusters + 1; i++)
				{
					if(index[i] == -1)
					{
						index[0] = i - 1;
						index[i] = INT_MAX;
						break;
					}
				}
			}
			
			//~ printf("%s %2.3f\n","Depth:  " ,median);
			
			Mat src_gray;
			depthToGray(src, src_gray, 0, max_depth);
			for(int i = 0; i < src_gray.rows; i++)
			{
				uchar* cur = src_gray.ptr<uchar>(i);	
				for(int j = 0; j < src_gray.cols; j++)
				{
					if(rep.cidx[i*src_gray.cols + j] == index[0])
					{
						cur[j] = 0;
					}
					else
					{
						cur[j] = 255;
					}
				}
			}
			
			Mat temp_img(depth_height, depth_width, CV_8UC1);
			temp_img = Scalar(0);

			src.copyTo(temp_img(Rect(personRect.x, personRect.y, src.cols, src.rows)));
			imshow("Clustered", temp_img);
			moveWindow("Clustered", 645, 0);
			
		
		}
		
	}
	return median;
	
	
	/*
	// Preprocessing step to discard null or unwanted depth values 
	// and convert Mat to vector
	for(int i = 0; i < rows; i++)
	{
		float* cur = src.ptr<float>(i);
		for(int j = 0; j < cols; j++)
		{
			vec.push_back(cur[j]);
		}
		
	}
	// average of minimum vector elements 
	saveMin = minDepth(vec, 20);

	// find the depth of the mat center area 
	saveCenter = centerDepth(src, 10);
	
	
	
	
	//~ int id = -1;
	//~ double result = FLT_MAX;
	//~ 
	//~ for(int i = 0; i < rep.c.rows() - 1; i++)
	//~ {
		//~ double temp = std::min(result, rep.c[i + 1][0]);
		//~ if(temp > 0)
		//~ {
			//~ id = i;
			//~ result = temp;
		//~ }
	//~ }
	//~ saveCluster = result;
	
	//combine the results of the 2 closest
	depthCombined = combineDepth(saveMin, saveCenter, median, 0, max_depth);
	

	//~ printf("%s %2.3f\n","Depth combined:  " ,depthCombined);
	return depthCombined;
	*/
		
}

/* Finds an average of the minimum 
 * values of a given vector
 * 
 * PARAMETERS:
 * 		-vector with the values
 * 		-number of values to calculate
 * 		
 * RETURN:   
 * 		-Double holding the average value
 * 
 */
double Fusion_processing::minDepth(vector<double> vec, int number)
{
	double result = 0;
	double min = 0;
	int done = 0;
	float threshold = number;
	partial_sort (vec.begin(), vec.begin() + threshold, vec.end());
	for(int i = 0; i < threshold; i++)
	{
		result += vec.at(i);
	}
	result = result/threshold;
	cout<<"Save Min:  "<<result<<endl;
	return result;
}

/* Finds an average of a central subsector of a Mat
 * 
 * PARAMETERS:
 * 			-Mat with the values
 * 			-number to from the area
 * 		
 * RETURN:
 *  		-Double holding the average value
 * 
 */
double Fusion_processing::centerDepth(const Mat& src, int number)
{
	int width, height, counter = 0;
	double result, temp;
		
	
	for(int i = 0; i < number; i++)
	{
		for(int j = 0; j < number; j++)
		{
			temp = src.at<float>(src.rows/2 - number/2 + i, src.cols/2 - number/2 + j);
			if(std::isnan(temp) || temp == FLT_MAX)
				continue;
			result += temp;
			counter++;
		}
	}
	
	cout<<"Save Center:  "<<result/counter<<endl;
	//~ if(result/counter < min_depth || result/counter > max_depth)
	//~ {
		//~ return FLT_MAX;
	//~ }
	return result/counter;
}

/* Clusterizes a given vector
 * 
 * PARAMETERS:
 * 			-Vec with the values
 * 			-Int with the number of clusters
 * 
 * 	RETURN:
 * 			-Double holding the minimum cluster value
 * 
 */
kmeansreport Fusion_processing::clusterize(const vector<float>& vec, int clusters)
{
	clusterizerstate s;
	kmeansreport rep;
	
	real_2d_array matrix;
	matrix.setlength(vec.size(), 1);
	
	for (int i = 0; i < vec.size(); i++)
	{
			matrix(i, 0) = vec.at(i);
	}
	
	try
	{
		clusterizercreate(s);
		clusterizersetpoints(s, matrix, 2);
		clusterizersetkmeanslimits(s, 5, 0);
		clusterizerrunkmeans(s, clusters, rep);
	}
	catch(exception& e)
	{
		printf("%s %s", "Clusterize failed", e.what());
	}
	return rep;
	    

}

/* Calculates an average of the 2 closest values
 * 
 * PARAMETERS:
 * 			-Double the min depth approximation
 * 			-Double the center depth approximation
 * 			-Double the cluster depth approximation
 * 
 * 
 * 	RETURN:
 * 			-Double holding the average of the 2 closest values
 */
double Fusion_processing::combineDepth(double saveMin, double saveCenter, double saveCluster, double min_depth, double max_depth)
{
	double result;
	double minClusterDif;
    double minCenterDif;
    double clusterCenterDif;
    
    
    
    
	if((saveMin == FLT_MAX || saveMin == 0) && (saveCluster == FLT_MAX || saveCluster == 0))
		minClusterDif = FLT_MAX;
	else
		minClusterDif = abs(saveMin - saveCluster);
		
	if((saveMin == FLT_MAX || saveMin == 0) && (saveCenter == FLT_MAX || saveCenter == 0))
		minCenterDif = FLT_MAX;
	else
		minCenterDif = abs(saveMin - saveCenter);
		
	if((saveCluster == FLT_MAX || saveCluster == 0) && (saveCenter == FLT_MAX || saveCenter == 0))
		clusterCenterDif = FLT_MAX;
	else
		clusterCenterDif = abs(saveCluster - saveCenter);
			
		
    if(minClusterDif > max_depth/10)
		minClusterDif = FLT_MAX;
    if(minCenterDif > max_depth/10)
		minCenterDif = FLT_MAX;
    if(clusterCenterDif > max_depth/10)
		clusterCenterDif = FLT_MAX;
		
    if(minClusterDif < minCenterDif)
    {
		if(minClusterDif < clusterCenterDif)
			result = (saveMin + saveCluster)/2;
		else
			result = (saveCluster + saveCenter)/2;
	}
    else if(minCenterDif < clusterCenterDif)
		result = (saveMin + saveCenter )/2;
	else if(clusterCenterDif < minClusterDif)
		result = (saveCenter + saveCluster)/2;
	else
	{
		result = max(saveMin, saveCenter);
		result = max(result, saveCluster);
	}
	
	 
	return result;
}

		
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  Fusion_processing fp;
  ros::spin();
  return 0;
}


		
	
