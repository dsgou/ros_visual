#include <utility.hpp>

Utility::Utility()
{
}

Utility::~Utility()
{
}

string Utility::create_directory(string path_, bool create_csv, vector<string> fields, bool save_images)
{
	
	string session_path = path_ + "/sessions/";
	
	//~ Getting current time
	string time = getTime("%d-%m-%Y_%X");
    
	//~ Creating session directory
	boost::filesystem::detail::create_directory(session_path);
	
	session_path = (session_path + time);
	boost::filesystem::detail::create_directory(session_path);
	
	if(create_csv)
	{
		ofstream storage(session_path + "/fusion.csv", ios::out | ios::app );
        for(string f: fields)
            storage<<f<<"\t";
            
        storage<<endl;
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


