#include <decision.hpp>


Decision_making::Decision_making()
{
	 //Getting the parameters specified by the launch file 
	ros::NodeHandle local_nh("~");
	local_nh.param("camera_frame" 	 , camera_frame	   , string("camera_link"));
	local_nh.param("results_topic"   , results_topic   , string("results"));
	local_nh.param("events_topic"    , events_topic    , string("events"));
	local_nh.param("project_path"    , path_           , string(""));
	local_nh.param("csv_fields"      , csv_fields      , string(""));
	local_nh.param("max_depth"       , max_depth       , DEPTH_MAX);
	local_nh.param("min_depth"       , min_depth       , DEPTH_MIN);
	local_nh.param("history_size"    , history_size    , 7);
	local_nh.param("create_directory", create_dir      , false);
	local_nh.param("write_csv"       , write_csv       , false);

    fusion_sub = nh_.subscribe(results_topic, 1, &Decision_making::callback, this);
    results_publisher = local_nh.advertise<decision_making::Event>(events_topic, 1000);

    if(create_dir)
    {
        string temp;
        vector<string> fields;
        stringstream stream(csv_fields);
        while(stream >> temp)
        {
            replace(temp.begin(), temp.end(), ',', ' ');
            fields.push_back(temp);
        }
        session_path = create_directory(path_, write_csv, fields);
    }
	
}

void Decision_making::callback(const fusion::FusionMsg::ConstPtr& msg)
{
    decision_making::Event event_msg;
    event_msg.header.stamp = msg->header.stamp;
    event_msg.header.frame_id = camera_frame;
    event_msg.event = -1;
    
    for(vector<bool>::iterator it = inserted.begin(); it < inserted.end(); it++)
        *it = false;
        
    for(const fusion::Box_<std::allocator<void> > box_ :msg->boxes)
    {
        Decision_making::Box box;
        box.id           = box_.id;
        box.rect.x       = box_.rect.x;
        box.rect.y       = box_.rect.y;
        box.rect.width   = box_.rect.width;
        box.rect.height  = box_.rect.height;
        box.pos.x        = box_.pos.x;
        box.pos.y        = box_.pos.y;
        box.pos.z        = box_.pos.z;
        box.pos.top      = box_.pos.top;
        box.pos.height   = box_.pos.height;
        box.pos.distance = box_.pos.distance;
        
        if(boxes_hist.size() < box.id + 1)
        {
            vector<Decision_making::Box> temp_boxes;
            temp_boxes.push_back(box);
            boxes_hist.push_back(temp_boxes);
            inserted.push_back(true);
        }
        else
        {
            boxes_hist.at(box.id).insert(boxes_hist.at(box.id).begin(), box);
            if(boxes_hist.at(box.id).size() > history_size)
                boxes_hist.at(box.id).pop_back();
            inserted.at(box.id) = true;
        }
    }
    
    
    //Calculates the median distance and publishes the approximate time when
    //a person has walked 4 meters and the median_top and calculates when
    //he stands up
    int i = 0;
    for(vector<vector<Decision_making::Box>>::iterator it = boxes_hist.begin(); it < boxes_hist.end(); it++, i++)
    {
        vector<float> x_pos;
        vector<float> y_pos;
        vector<float> top_pos;
        for(Decision_making::Box box: *it)
        {
            x_pos.push_back(box.pos.x);
            y_pos.push_back(box.pos.y);
            top_pos.push_back(box.pos.top);
        }
        float x_median = median(x_pos);
        float y_median = median(y_pos);
        float top_median = median(top_pos);
        
        if(positions.size() < i + 1)
        {
            Decision_making::Position pos;
            pos.x = x_median;
            pos.y = y_median;
            pos.top = top_median;
            vector<Decision_making::Position> temp;
            temp.push_back(pos);
            positions.push_back(temp);
        }
        else
        {
            
            Decision_making::Position pos;
            pos = positions.at(i).front();
            float dist = sqrt(pow(x_median - pos.x, 2) + pow(y_median - pos.y, 2));
            float top_dif = abs(top_median - pos.top);
            
            if(top_dif > 3 && top_median > 200 && top_median < 900)
            {
                if(standUp_time.toSec() == 0)
                    standUp_time = event_msg.header.stamp;
            }
            else
            {
                if(standUp_time.toSec() > 0)
                {
                    float duration = event_msg.header.stamp.toSec() - standUp_time.toSec();
                    if((duration > 0.4) && (duration < 2.0))
                    {
                        ros::Time time = event_msg.header.stamp;
                        ofstream storage(session_path + "/decision_making.csv" ,ios::out | ios::app );
                        storage
                            <<standUp_time<<"\t"
                            <<i<<"\t"
                            <<"2\t"<<
                        endl;
                        storage
                            <<event_msg.header.stamp<<"\t"
                            <<i<<"\t"
                            <<"3\t"<<
                        endl;
                        event_msg.event = 3;
                        results_publisher.publish(event_msg);
                    }
                }
                standUp_time = ros::Time(0);
            }
            pos.x = x_median;
            pos.y = y_median;
            pos.top = top_median;
            positions.at(i).insert(positions.at(i).begin(), pos);
            
            if(positions.at(i).size() > history_size)
                positions.at(i).pop_back();
            
            if(distances.size() < i + 1)
            {
                vector<float> temp;
                temp.push_back(dist);
                distances.push_back(temp);
                vector<Decision_making::Box> temp_boxes = *it;
                temp_boxes.front().pos.acc_distance = dist;
                *it = temp_boxes;
            }
            else
            {
                float dist_median = median(distances.at(i));
                vector<Decision_making::Box> temp_boxes = *it;
                if(temp_boxes.size() > 1)
                {
                    float prev_dist = temp_boxes.at(1).pos.acc_distance;
                    dist += prev_dist;
                    temp_boxes.front().pos.acc_distance = dist;
                    if(dist > 4000 && prev_dist < 4000 && write_csv)
                    {
                        ros::Time time = event_msg.header.stamp;
                        ofstream storage(session_path + "/decision_making.csv" ,ios::out | ios::app );
                        storage
                            <<time<<"\t"
                            <<i<<"\t"
                            <<"4\t"<<
                        endl;
                        event_msg.event = 4;
                        results_publisher.publish(event_msg);
                    }
                }
                else
                    temp_boxes.front().pos.acc_distance = dist;
                *it = temp_boxes;
                distances.at(i).insert(distances.at(i).begin(), dist_median);
                if(distances.at(i).size() > history_size)
                    distances.at(i).pop_back();
            }
            
            
            
        }
    }
        
    vector<vector<float>>::iterator dist_it                    = distances.begin();
    vector<vector<Decision_making::Box>>::iterator boxes_it    = boxes_hist.begin();
    vector<vector<Decision_making::Position>>::iterator pos_it = positions.begin();
    for(vector<bool>::iterator bool_it = inserted.begin(); bool_it < inserted.end(); bool_it++, boxes_it++, dist_it++, pos_it++)
    {
        bool flag  = *bool_it;
        if(!flag)
        {
            vector<Decision_making::Box> temp_boxes = *boxes_it;
            temp_boxes.pop_back();
            *boxes_it = temp_boxes;
            if(temp_boxes.empty())
            {
                boxes_it = boxes_hist.erase(boxes_it);
                bool_it = inserted.erase(bool_it);
                dist_it = distances.erase(dist_it);
                pos_it = positions.erase(pos_it);
            }
        
        }
    }
        
    
    
    //~ for(vector<float> dist: distances)
    //~ {
        //~ for(float dist_value: dist)
        //~ {
            //~ cout<<dist_value<<endl;
        //~ }
        //~ cout<<endl;
    //~ }
    //~ for(vector<vector<Decision_making::Box>>::iterator it = boxes_hist.begin(); it < boxes_hist.end(); it++)
    //~ {
        //~ for(Decision_making::Box box: *it)
        //~ {
            //~ cout<<box.pos.acc_distance<<endl;
        //~ }
        //~ cout<<endl;
    //~ }
    //~ cout<<endl;
}

string Decision_making::create_directory(string path_, bool create_csv, vector<string> fields)
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
		ofstream storage(session_path + "/decision_making.csv", ios::out | ios::app );
        for(string f: fields)
            storage<<f<<"\t";
            
        storage<<endl;
		storage.close();
	}
	return session_path;
    
}

string Decision_making::getTime(string format)
{
	
	time_t now = time(0);
    gmtm = *localtime(&now);
    char buf[80];
	strftime(buf, sizeof(buf), format.c_str(), &gmtm);
	return buf;
}

float Decision_making::median(vector<float> values)
{
    float median = 0.0;
    if(!values.empty())
    {
        sort(values.begin(), values.end());
        if(values.size() == 1)
        {
            median = values.front();
        }
        else if(values.size()%2 == 0)
        {
            median = (values.at(values.size()/2 - 1) + values.at(values.size()/2))/2;
        }
        else
        {
            median = values.at(values.size()/2);
        }
    }
    return median;
    
}
		
int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_making");
  Decision_making dm;
  ros::spin();
  return 0;
}


		
	
