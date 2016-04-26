#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <vector>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <string>
#include <math.h>
#include <cmath>
#include <sys/stat.h>
#include <ctime>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace cv;

class Utility
{
	public:
	
		Utility();
		~Utility();
		
		string initialize(string path_, bool create_csv, bool save_images);
		void storeImage(Mat image, string session_path);
		void storeDepth(Mat image, string session_path);
		string getTime(string format);
	
	private:
		struct tm gmtm;
};

#endif // UTILITY_HPP
