#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include <cmath>
#include <ctime> 
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;

class Utility
{
	public:
	
		Utility();
		~Utility();
		
		string create_directory(string path_, bool create_csv, vector<string> fields, bool save_images);
		void storeImage(Mat image, string session_path);
		void storeDepth(Mat image, string session_path);
		string getTime(string format);
	
	private:
		struct tm gmtm;
};

#endif // UTILITY_HPP
