#include <chroma.hpp>


	
Chroma_processing::Chroma_processing()
: it_(nh_)
{
	//Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("image_topic", image_topic, string("/camera/rgb/image_raw"));
	local_nh.param("image_out_topic", image_out_topic, string("/chroma_proc/image_raw"));
	local_nh.param("project_path",path_, string(""));
	local_nh.param("playback_topics", playback_topics, false);
	local_nh.param("display", display, false);
	
	if(playback_topics)
	{
		ROS_INFO_STREAM_NAMED("Chroma_processing","Subscribing at compressed topics \n"); 
		
		image_sub = it_.subscribe(image_topic, 1, 
		  &Chroma_processing::imageCb, this, image_transport::TransportHints("compressed"));
    }
    else
    {
		// Subscribe to input video feed 
		image_sub = it_.subscribe(image_topic, 1, &Chroma_processing::imageCb, this);
		
	} 
	
	image_pub = it_.advertise(image_out_topic, 1); 
}

Chroma_processing::~Chroma_processing()
{
	//destroy GUI windows
	destroyAllWindows();
	
}



/* Callback function to handle ros image messages
 * 
 * PARAMETERS:
 * 			-the pointer that contains the color image 
 * 			 and its metadata
 * 
 * RETURN: --
 */
void Chroma_processing::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	
	
	int rows;
	int cols;
	int channels;
	int size;
	
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
	cur_rgb.convertTo(cur_rgb, -1, 4, 0);
	
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
	
	//~ medianBlur(dif_rgb, dif_rgb, 3);
	frameDif(cur_rgb, ref_rgb, dif_rgb, 255*0.33);
	ref_rgb = cur_rgb.clone();

	
	//Display
	 
	//Blob detection
	if(display)
	{
		detectBlobs(dif_rgb, rgb_rects, 15);
		
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
	waitKey(1);
	
	cv_ptr->header.stamp = ros::Time::now();
	cv_ptr->image = dif_rgb;
	image_pub.publish(cv_ptr->toImageMsg());
}

/* Detects non-black areas in the image and populates a list of OpenCV Rects  
 *
 * PARAMETERS: -the vector that will be populated
 * 			   -screen width
 * 
 * RETURN --
 */
void Chroma_processing::detectBlobs(const Mat& src, vector< Rect_<int> >& colour_areas, int range)
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
void Chroma_processing::track(vector< Rect_<int> >& cur_boxes, People& collection, int rank, float threshold)
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

/* Estimates the foreground by combining static images, works for
 * images the contain edge information
 * 
 * @param a mat representing the current image frame, a mat to store the result,
 *  the number of images to combine 
 * @return -
 */
void Chroma_processing::estimateForeground(Mat& cur_Mat, Mat& back_Mat, Mat& dst_Mat)
{
	uchar *back, *cur, *dst;
	for(int y = 0; y < 1; y++)
	{
		int rows	 = cur_Mat.rows;
		int cols 	 = cur_Mat.cols;
		int channels = cur_Mat.channels();
		int size 	 = rows*cols*channels;
	
		back = back_Mat.ptr<uchar>(y);
		cur = cur_Mat.ptr<uchar>(y);
		dst = dst_Mat.ptr<uchar>(y);
		for(int x = 0; x < size; x = x + channels)
		{ 
			if(back[x] != 0)
			{
				dst[x] = 0;
			}
		}
	}
}
	
/* Estimates the background by combining static images, works for
 * images the contain edge information
 * 
 * @param a mat representing the current image frame, a mat to store the result,
 *  the number of images to combine 
 * @return -
 */
void Chroma_processing::estimateBackground(Mat& src, Mat& dst, vector<Mat>& storage, int recursion, float ratio, int index)
{
	int size = storage.size();
	Mat result;
	if (index > recursion)
	{
		dst = storage.at(index - 1);
		return;
	}
	if(size > 0)
	{
		if((index <= 0) || ((index >= recursion*ratio && (size >= recursion*ratio))))
		{
			bitwise_or(src, storage.at(index), result);
		}
		else
			bitwise_and(src, storage.at(index), result);
		if((size -1) == index)
		{
			storage.push_back(result);
			dst = result;
		}
		else
		{
			storage.at(index) = src;
			index++;
			estimateBackground(result, dst, storage, recursion, ratio, index);	
		}
	}
	else
	{
		storage.push_back(src);
		dst = src.clone();
	}
}

/*Calculates the absolute difference between the two mats and thresholds the result according to the threshold given
 * 
 * PARAMETERS:
 * 			-Mat first
 * 			-Mat second
 * 			-Mat with the result
 * 			-float threshold to be used
 * 
 * RETURN: --
 */
void Chroma_processing::frameDif(Mat& src1, Mat& src2, Mat& dst, float threshold)
{
	uchar *src, *temp, *dest;
	int channels = src1.channels();
	int cols = src1.cols;
	int rows = src1.rows;
	int size = cols*rows*channels;
	int motionCounter = 0;
	
	Mat temp_Mat;
	// Absolute dif between our current mat and the previous one 
	absdiff(src1, src2, temp_Mat);
	dst = src1.clone();
	for(int y = 0; y < 1; y++)
	{
		src = src1.ptr<uchar>(y);
		temp = temp_Mat.ptr<uchar>(y);
		dest = dst.ptr<uchar>(y);
		for(int x = 0; x < size; x = x + channels)
		{ 
			float all = 0.0;
			float color[channels];
			for (int ch = 0; ch < channels; ch++)
				color[ch] = (float)temp[x + ch];
				
			for (int i = 0; i < channels; i++)
				all += color[i];
			all /= channels;
			
			if(all > threshold)
			{
				for (int ch = 0; ch < channels; ch++)
					dest[x + ch] = src[x + ch]; 
				motionCounter++;
			}
			else
			{
				for (int ch = 0; ch < channels; ch++)
					dest[x + ch] = 0;
			}
		}
	}
}

/* Thresholds an image to zero (i > thresh -> 0) 
 *
 * @param the Mat to be processed, the threshold to use
 * @return the number of pixels above the threshold
 */
int Chroma_processing::threshold(Mat& src, Mat& dst, int thresh)
{	
	
	int motionCounter = 0;
	int end = src.cols*src.rows*src.channels();
	int step = src.channels();
	for(int y = 0; y < 1; y++)
	{
		uchar *cur = src.ptr<uchar>(y);
		uchar *temp = dst.ptr<uchar>(y);
		for(int x = 0; x < end; x = x + step)
		{ 
			double bPercent =  double(cur[x]) ;
			if(bPercent > thresh)
				motionCounter++;
			else 
			{
				temp[x] = 0;
				temp[x + 1] = 0;
				temp[x + 2] = 0;
			}
		}
		
	}
	return motionCounter;

}

/* Image gamma correction 
 *
 * @param the Mat to be processed
 * @return -
 */
void Chroma_processing::gammaCorrection(Mat& src)
{
	double inverse_gamma = 1.0 / 2.2;
	
	Mat lut_matrix(1, 256, CV_8UC1 );
	uchar * ptr = lut_matrix.ptr();
	for( int i = 0; i < 256; i++ )
		ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
	LUT( src, lut_matrix, src );
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "chroma");
	Chroma_processing ip;
	ros::spin();	
	return 0;
}
