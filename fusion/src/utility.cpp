#include <utility.hpp>

Utility::Utility()
{
}

Utility::~Utility()
{
}

string Utility::initialize(string path_, bool create_csv, bool save_images)
{
	
	string session_path = path_ + "/sessions/";
	
	//~ Getting current time
	string time = getTime("%d-%m-%Y_%X");
	
	//~ Creating session directory
	boost::filesystem::detail::create_directory(session_path);
	
	session_path = (session_path + time);
	boost::filesystem::detail::create_directory(session_path);
	
	
	if(save_images)
	{
		boost::filesystem::detail::create_directory(session_path + "/images/");
		boost::filesystem::detail::create_directory(session_path + "/depth/");
	
	}
	if(create_csv)
	{
		ofstream storage(session_path + "/session.csv", ios::out | ios::app );
		storage<<"Timestamp\tRect_id\tRect_x\tRect_y\tRect_W\tRect_H\tMeter_X\tMeter_Y\tMeter_Z\tTop\tHeight\tDistance"<<endl;
		storage.close();
	}
	return session_path;
    
}

//~ Store rgb image
void Utility::storeImage(Mat image, string session_path)
{
	
    string time = getTime("%X");
	imwrite(session_path + "/images/" + time + ".png", image);
}


//~ Store depth image
void Utility::storeDepth(Mat depth, string session_path)
{
	string time = getTime("%X");
	imwrite(session_path + "/depth/" + time + ".png", depth);
}

string Utility::getTime(string format)
{
	
	time_t now = time(0);
    gmtm = *localtime(&now);
    char buf[80];
	strftime(buf, sizeof(buf), format.c_str(), &gmtm);
	return buf;
}


