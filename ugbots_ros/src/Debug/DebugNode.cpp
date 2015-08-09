class DebugNode
{
public:
	DebugNode(ros::NodeHandle &n)
		{
			this->n = n;

			sub_list.sub_debug = n.subscribe</*Create topic type**/>();
		}


		void dispEvents(/*Create topic type**/){

			ROS_INFO(/*display info here**/)
		}
};

int main(int argc, char **argv)
{	

	//Initialise the robot parameters
	ros::init(argc, argv, "d1");
	
	//Nodehandle is the main access point to communicate with ros.
	ros::Nodehandle n;

	DebugNode d1(n);
	
	return 0;
}
