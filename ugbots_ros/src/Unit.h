class Unit

{
public:
	void publish()
	{
	
	}

	virtual void moveTo(int x, int y){}
	virtual void stop (){}
	virtual void collisionDetected(){}

	//pose of the unit
	double px;
	double py;
	double theta;

	//velocity of the unit
	double linear_x;
	double angular_z;

	//max velocity of the unit	
	double max_linear_x;

	//NodeHandle for the node
	ros::NodeHandle n;

	//Velocity of the robot
	geometry_msgs::Twist node_cmdvel;

	//Odometry subscriber
	ros::Subscriber sub_odom;
	//Laser subscriber
	ros::Subscriber sub_laser;
	//Publisher for publishing to stage
	ros::Publisher node_stage_pub;
}
