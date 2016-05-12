#include <utility.hpp>

Utility::Utility()
{
}

Utility::~Utility()
{
}

string Utility::initialize(string path_, bool create_csv, bool save_images)
{
	
	char const *pchar = (path_ + "/sessions/").c_str();
	string session_path = pchar;
	
	//~ Getting current time
	string time = getTime("%d-%m-%Y_%X");
	
	//~ Creating session directory
	mkdir(pchar,  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	session_path = (session_path + time);
	int result = mkdir(session_path.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	if(save_images)
	{
		pchar = (session_path + "/images/").c_str();
		result += mkdir(pchar,  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		
		pchar = (session_path + "/depth/").c_str();
		result += mkdir(pchar,  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	}
	if(create_csv)
	{
		
		pchar = (session_path + "/session.csv").c_str();  
		ofstream storage;
		storage.open (pchar,ios::out | ios::app );
		storage<<"Timestamp\tRect_id\tRect_x\tRect_y\tRect_W\tRect_H\tMeter_X\tMeter_Y\tMeter_Z\tTop\tHeight\tDistance"<<endl;
		storage.close();
	}
	return (session_path).c_str();
    
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


