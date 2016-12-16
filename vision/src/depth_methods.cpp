#include <vision.hpp>

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
void grayToDepth(Mat& src, Mat& dst, float max_depth)
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
void depthToGray(Mat& src, Mat& dst, float min_depth, float max_depth)
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

/* Calculates and stores the coordinates(x, y, z) in meters of a rectangle
 * in respect to the center of the camera.
 * 
 * PARAMETERS:
 * 			- rect   : rectangle to be processed
 * 			- pos    : object to save the measurements produced
 * 			- width  : image width
 * 			- height : image height
 * 			- Hfield : camera horizontal field of view in degrees
 * 			- Vfield : camera vertical field of view in degrees
 * 
 * RETURN: --
 * 
 */
void calculatePosition(Rect& rect, Position& pos, int width, int height, int Hfield, int Vfield)
{
	
	float hor_x 	= 0.0;
	float hor_y 	= 0.0;
	float ver_x 	= 0.0;
	float ver_y 	= 0.0;
	float hor_focal = 0.0;
	float ver_focal = 0.0;
	float distance  = 0.0;
	float top 		= 0.0;
	float bottom 	= 0.0;
	float depth 	= pos.z;
	
	if (depth != 0.0)
	{
		
		//Find the focal length 
		hor_focal = height / (2 * tan((Vfield/2) * M_PI / 180.0) );
		ver_focal = width / (2 * tan((Hfield/2) * M_PI / 180.0) );
		
		//Transform the pixel x, y in respect to 
		//the camera center
		ver_x = rect.x + rect.width/2;
		if(ver_x > width/2)
			ver_x = (ver_x - width/2);
		else
			ver_x = (-1)*(width/2 - ver_x);
		
		ver_y = rect.y + rect.height/2;
		if(ver_y > height/2)
			ver_y = (-1)*(ver_y - height/2);
		else
			ver_y = (height/2 - ver_y);
		
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
		if(ver_y > height/2)
			ver_y = (-1)*(ver_y - height/2);
		else
			ver_y = (height/2 - ver_y);
		top = depth * ver_y / hor_focal;
		
		
		ver_y = rect.y + rect.height;
		if(ver_y > height/2)
			ver_y = (-1)*(ver_y - height/2);
		else
			ver_y = (height/2 - ver_y);
			
		bottom     = depth * ver_y / hor_focal;
		pos.top    = abs(top);
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
double calculateDepth(Mat& src, Rect_<int> personRect)
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
	Mat labels;
	Mat centers;
	
	if(cols > 0 && rows > 0)
	{
		
		if(src.isContinuous())
		{
			rows = 1;
			cols = size;
		}
		
		cols = src.cols*0.7;
		rows = src.rows*0.4;
		
		for(int i = 0; i < rows; i++)
		{
			const float* cur = src.ptr<float>(i);
			for(int j = 0; j < cols; j++)
			{
				cluster_vec.push_back(cur[j]);
				
			}
			
		}
		double compact = kmeans(cluster_vec, clusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
		
		// cluster item count
		int counter[clusters];
		for(int i = 0; i < clusters; i++)
		{
			counter[i] = 0;
		}
		
		//check which centroid has the smallest value
		int min_centroid = INT_MAX;
		int index [clusters + 1];
		for(int i = 0; i < clusters + 1; i++)
		{
			index[i] = -1;
		}
		
		if(centers.rows > 0)
		{	
			for(int i = 0; i < clusters; i++)
			{
				if((min_centroid > centers.at<float>(i)) && (centers.at<float>(i) > 0))
				{
					min_centroid = centers.at<float>(i);
					index[0] = i;
				}
			}
			index[index[0] + 1] = INT_MAX;
			
			//cout<<rep.c[0][0] <<" "<< rep.c[1][0]<<" "<<min_centroid<< " " <<index[0]<<endl;
			//~ cout<<counter[0] <<" "<< counter[1]<<" "<<endl;
			
			
			//populate a vector with the points of the cluster
			//with the smallest centroid and find the median value
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
						if(index[0] == labels.at<int>(i*cols + j))
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
			
			/*
			Mat src_gray;
			depthToGray(src, src_gray, 0, max_depth);
			for(int i = 0; i < rows; i++)
			{
				uchar* cur = src_gray.ptr<uchar>(i);	
				for(int j = 0; j < cols; j++)
				{
					if(rep.cidx[i*cols + j] == index[0])
					{
						cur[j] = 0;
					}
					else
					{
						cur[j] = 255;
					}
				}
			}
			
			Mat temp_img(height, depth_width, CV_8UC1);
			temp_img = Scalar(0);

			src.copyTo(temp_img(Rect(personRect.x, personRect.y, cols, rows)));
			imshow("Clustered", temp_img);
			moveWindow("Clustered", 645, 0);
			*/
		
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
double minDepth(vector<double> vec, int number)
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
double centerDepth(const Mat& src, int number)
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
double combineDepth(double saveMin, double saveCenter, double saveCluster, double min_depth, double max_depth)
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

/*Region growing algorithm that fills black holes in the depth image, uses
 * rectangular areas
 * 
 * PARAMETERS:
 * 			-Mat to be corrected
 * 			-float threshold of correction
 * 			-int range of each correctin area
 * 
 * RETURN: --
 */
void rectFill(Mat& src, float threshold, int range)
{
	uchar *cur, *temp,*vFill;
	int channels = src.channels();
	int cols = src.cols;
	int rows = src.rows;
	int size = cols*rows*channels;
	int count = 0;
	float all = 0.0;
	float bPercent = 0.0;
	float gPercent = 0.0;
	float rPercent = 0.0;
	bool has_zero = false;
	bool flag = false;
	
	for(int y = 0; y < 1; y++)
	{
		cur = src.ptr<uchar>(y);
		for(int x = channels*range + cols*range; x <  size - (channels*range + cols*range); x = x + channels)
		{
			if ((cur[x] != 0 && channels == 1) || (channels > 1 && (cur[x+1] != 0 || cur[x+2] != 0) ))
			{
				int top 		= x - range*cols*channels;
				int topLeft 	= top - range*channels;
				int topRight 	= top + range*cols*channels;
				int bottom 		= x + range*cols*channels;
				int bottomLeft  = bottom - range*cols*channels;
				int bottomRight = bottom + range*cols*channels;
				
				if(topLeft%(cols*channels)  > top%(cols*channels))
					continue;
				if(topRight%(cols*channels) < top%(cols*channels))
					continue;
				
				has_zero = false;
				flag = false;
				count = 0;
				for(int a = topLeft; a < bottomLeft; a = a + cols*channels)
				{
					for(int b = a; b < a + 2*range*channels ; b += channels)
					{
						if (cur[b] == 0)
						{
							has_zero = true;
							count++;
						}
						else
						{
							if(channels == 1)
							{
								all = abs( (float(cur[b])/255)  - (float(cur[x])/255) );
								if(all >= threshold)
									flag = true;
							}
						}
							
					}
				}
				int countThresh = range;
				if(!flag && has_zero && (count <= countThresh))
				{
					for(int a = topLeft; a <  bottomLeft; a = a + cols*channels)
					{
						for(int b = a; b < a + 2*range*channels ; b += channels)
						{
							if (cur[b] == 0)
							{
								cur[b] = cur[x];
							}
						}
					}
				}
				//~ x = x - range*channels + channels;
			}
		}
	}
}

/* Fills the black holes in the Mat horizontally    
 * 
 * PARAMETERS:
 * 			-the Mat to be processed
 * 			-the comparison threshold 
 * 
 * RETURN: --
 * 
 */
void rightHorizontalFill(Mat& cur_Mat, float threshold, bool scale)
{
	uchar *cur;
	int channels = cur_Mat.channels();
	int cols = cur_Mat.cols;
	int rows = cur_Mat.rows;
	int size = cols*rows*channels;
	float all = 0.0, bPercent = 0.0, gPercent = 0.0, rPercent = 0.0;
	
	for(int y = 0; y < 1; y++)
	{
			cur = cur_Mat.ptr<uchar>(y);
			for(int x = 0; x < size; x = x + channels)
			{
				/*If it is the end of the current line, continue*/
				if(x > 0 && (x%(cols*channels) == 0 || x%(cols*channels) == (cols - 1)))
					continue;
				if(cur[x] != 0 && ((cur[x + channels] == 0 && channels == 1) || (channels > 1 && (cur[x+1] != 0 | cur[x+2] != 0) )))
				{
					int start = x + 2*channels;
					for(int a = start; a < size; a = a + channels)
					{
						//check not to change line
						if((x%(cols*channels)) > (a%(cols*channels)))	
							break;	
						if(cur[a] != 0)
						{
							if(channels == 1)
							{
								all = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
								if(all < threshold)
								{
									if(scale)
									{
										int rowA = a%(cols*channels);
										int rowX = x%(cols*channels);
										all = (all/abs(rowA - rowX))*255;
										int ratio = 1;
										for(int i = x + channels; i < a; i = i + channels)
										{
											cur[i] = cur[x] + ratio*all;
											ratio++;
										}
									}
									else
									{
										for(int i = x + channels; i < a; i = i + channels)
											cur[i] = (cur[x] + cur[a])/2;
									}
								}
							}
							else if(cur[a + 1] == 0 && cur[a + 2] == 0)
							{
							
							
								bPercent = abs( (double(cur[a])/255)  - (double(cur[x])/255) );
								gPercent = abs( (double(cur[a + 1])/255) - (double(cur[x + 1])/255) );
								rPercent = abs( (double(cur[a + 2])/255) - (double(cur[x + 2])/255) );
								
								all = bPercent + gPercent + rPercent;
								
								if(all < threshold){
									cur[a] = cur[x];
									cur[a+1] = cur[x+1];
									cur[a+2] = cur[x+2];
								}
								else
									break;
							}
							break;	
						}
					}
				}
			}
		}
}
 
/* Fills the black holes in the Mat vertically 
 * from bottom to top     
 * 
 * @param the Mat to be processed, the comparison threshold 
 * @return -
 * 
 */
void upVerticalFill(Mat& src, float threshold, bool flag)
{
	uchar *cur;
	int channels = src.channels();
	int cols = src.cols;
	int rows = src.rows;
	int size = cols*rows*channels;
	float all = 0.0;
	float bPercent = 0.0;
	float gPercent = 0.0;
	float rPercent = 0.0;
	
	threshold *= 255;
	float threshold2 = 0.01*255;
	for(int y = 0; y < 1; y++)
	{
		cur = src.ptr<uchar>(y);
		for(int x = size; x > cols*channels; x = x - channels)
		{
			
			if(int(cur[x]) != 0 && ((int(cur[x - cols*channels]) == 0 && channels == 1) || (channels > 1 && (cur[x+1] != 0 | cur[x+2] != 0) )))
			{
				
				int start = ((x - 2*cols*channels) > 0)?(x - 2*cols*channels):0;
				//~ int end = ((start + cols*verRange) < size)?(start + cols*verRange):size;
				
				for(int a = start; a > 0 ; a = a - cols*channels)
				{
					if((x - a)/(cols*channels) > rows/10)
					{
						break;
					}
					if(int(cur[a]) != 0)
					{
						if(channels == 1)
						{
							all = abs(int(cur[a])  - int(cur[x]));
							if(all < threshold)
							{
								int ratio = 1;
								int rowA = floor(a/(cols*channels));
								int rowX = floor(x/(cols*channels));
								int color = int(cur[x]) - int(cur[a]);
								color /= abs(rowA - rowX);
								
								for(int i = x - cols*channels; i > a; i = i - cols*channels)
								{
									cur[i] = int(cur[x]) - ratio*color;
									ratio++;
								}
							}
							else
							{
								for(int i = x - cols*channels; i > a; i = i - cols*channels)
								{
									int left = 0;
									int right = 0;
									for(int j = i; j < size ; j = j + channels)
									{
										//check not to change line
										if((i%(cols*channels)) > (j%(cols*channels)))	
											break;
										if(cur[j] != 0)	
										{
											if(channels == 1)
											{
												right = j;
											}
											break;
										}
									}
									for(int j = i; j > 0 ; j = j - channels)
									{
										//check not to change line
										if((i%(cols*channels)) < (j%(cols*channels)))	
											break;
										if(cur[j] != 0)	
										{
											if(channels == 1)
											{
												left = j;
											}
											break;
										}
									}
									if(!flag)
									{
										if((abs(int(cur[x]) - int(cur[left])) < threshold))
										{
											cur[i] = int(cur[left]) - (int(cur[left]) - int(cur[x]))/abs(left - x);
										}
										else if((abs(int(cur[x]) - int(cur[right])) < threshold))
										{
											cur[i] = int(cur[right]) - (int(cur[right]) - int(cur[x]))/abs(right - x);
										}
									}
									else
									{
										//~ if(abs(int(cur[left]) - int(cur[right])) < threshold)
										//~ {
											cur[i] = int(cur[x]);
										//~ }
									}
									//~ else
									//~ {
										//~ cur[i] = int(cur[a]);
									//~ }
								}
							}
						}
						else if(cur[a + 1] == 0 && cur[a + 2] == 0)
						{
							bPercent = abs(cur[a]  - cur[x]);
							gPercent = abs(cur[a + 1] - cur[x + 1]);
							rPercent = abs(cur[a + 2] - cur[x + 2]);
							
							all = (bPercent + gPercent + rPercent)/3;
							
							if(all < threshold)
							{
								cur[x] = cur[a];
								cur[x+1] = cur[a+1];
								cur[x+2] = cur[a+2];
							}
							else
								break;
						}
						break;	
					}
				}
			
			}
		}
	}
}

/* Fills the black holes in the Mat vertically 
 * from bottom to top     
 * 
 * @param the Mat to be processed, the comparison threshold 
 * @return -
 * 
 */
void upVerticalFill2(Mat& src, float threshold, bool scale)
{
	uchar *cur;
	int channels = src.channels();
	int cols = src.cols;
	int rows = src.rows;
	int size = cols*rows*channels;
	int left;
	int right;
	int opposite;
	float all = 0.0;
	float bPercent = 0.0;
	float gPercent = 0.0;
	float rPercent = 0.0;
	
	for(int y = 0; y < 1; y++)
	{
		cur = src.ptr<uchar>(y);
		for(int x = size; x > cols*channels; x = x - channels)
		{
			opposite = 0;
			if(cur[x] != 0 && ((cur[x - cols*channels] == 0 && channels == 1) || (channels > 1 && (cur[x+1] != 0 | cur[x+2] != 0) )))
			{
				
				int start = ((x - 2*cols*channels) > 0)?(x - 2*cols*channels):0;
				
				for(int a = start; a > 0 ; a = a - cols*channels)
				{
					if(cur[a] != 0)	
					{
						if(channels == 1)
						{
							all = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
							if(all < threshold)
							{
								opposite = a;
							}
						}
						break;
					}
				}
				if(opposite == 0)
					continue;
				start = x - cols*channels;
				for(int k = start; k > opposite; k = k - cols*channels)
				{
					left = 0;
					right = 0;
					for(int a = k; a < size ; a = a + channels)
					{
						//check not to change line
						if((k%(cols*channels)) > (a%(cols*channels)))	
							break;
						if(cur[a] != 0)	
						{
							if(channels == 1)
							{
								all = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
								if(all < threshold)
								{
									right = a;
								}
							}
							else if(cur[a + 1] == 0 && cur[a + 2] == 0)
							{
								bPercent = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
								gPercent = abs( (float(cur[a + 1])/255) - (float(cur[x + 1])/255) );
								rPercent = abs( (float(cur[a + 2])/255) - (float(cur[x + 2])/255) );
								
								all = (bPercent + gPercent + rPercent)/3;
								
								if(all < threshold)
								{
									for(int i = start; i < a; i = i + channels)
									{
										cur[i] = cur[x];
										cur[i+1] = cur[x+1];
										cur[i+2] = cur[x+2];
										
									}
									
									
								}
							}
							break;
						}
					}
					for(int a = k; a > 0 ; a = a - channels)
					{
						//check not to change line
						if((k%(cols*channels)) < (a%(cols*channels)))	
							break;
						if(cur[a] != 0)	
						{
							if(channels == 1)
							{
								all = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
								if(all < threshold)
								{
									left = a;
								}
							}
							else if(cur[a + 1] == 0 && cur[a + 2] == 0)
							{
								bPercent = abs( (float(cur[a])/255)  - (float(cur[x])/255) );
								gPercent = abs( (float(cur[a + 1])/255) - (float(cur[x + 1])/255) );
								rPercent = abs( (float(cur[a + 2])/255) - (float(cur[x + 2])/255) );
								
								all = (bPercent + gPercent + rPercent)/3;
								
								if(all < threshold)
								{
									for(int i = start; i > a; i = i - channels)
									{
										cur[i] = cur[x];
										cur[i+1] = cur[x+1];
										cur[i+2] = cur[x+2];
									}
								}
							}
							break;
						}
					}
					if(left == 0 || right == 0)
						continue;
					if(abs(left - right) > 10 )
						continue;
					all = abs( (float(cur[left])/255)  - (float(cur[right])/255) );
					if(all > threshold)
					{
						continue;
					}	
					all = float(cur[left])  - float(cur[right]) ;
					all = (all/abs(left - right));
					int ratio = 1;
					for(int a = left + channels; a < right; a = a + channels)
					{
						if(scale)
						{
							cur[a] = cur[left] - ratio*all;
							ratio++;
							
						}
						else
						{
							cur[a] = cur[x];
						}
					}	
				
				}
			}
		}
	}
}
