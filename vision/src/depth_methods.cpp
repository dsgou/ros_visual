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
float calculateDepth(const Mat& src, Position& pos)
{
	Mat labels;
	Mat centers;
	int clusters = 3;
	int attempts = 3;
	int j = 0;
	float depth= 0.0;
	float dif = 1000.0;
	float temp_depth = 0.0;
	int occur[clusters];
	int row_start = src.rows/4;
	int col_start = src.cols/4;
	
	Mat samples(4*row_start * col_start, 1, CV_32F);
	for( int y = 0; y < 2*row_start; ++y)
		for( int x = 0; x < 2*col_start; ++x)
			samples.at<float>(y + 2*x*row_start) = src.at<float>(y + row_start ,x + col_start);
	double c = kmeans(samples, clusters, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 10), attempts, KMEANS_PP_CENTERS, centers);
	for(int j = 0; j < labels.rows; ++j)
		++occur[labels.at<int>(j)];
	
	while(j < clusters && (temp_depth < 1000.0 || dif > 1000))
	{
		auto it = max_element(occur, occur + clusters);
		int index = distance(occur, it);
		temp_depth = centers.at<float>(index);
		occur[index]= 0;
		if(pos.z > 0)
			dif = abs(pos.z - temp_depth);
		else
			dif = 0.0;
		++j;
	}
	if (j < clusters)
		depth = temp_depth;
		
	return depth;
	
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
