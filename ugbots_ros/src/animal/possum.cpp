#include <node_defs/possum.h>

Possum::Possum()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 0;
	this->pose.py = 0;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	this->state = IDLE;

	this->row = 1; //starting at vine 1
	this->direction = EAST;

	this->initial_coordinates_set = false;	
}

Possum::Possum(ros::NodeHandle &n)
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 0;
	this->pose.py = 0;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	//register with neccessary topics
	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Possum::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Possum::laser_callback, this);

	//setup initial state.
	this->state = IDLE;
	this->row = 1; //starting at vine 1
	this->direction = EAST;
	this->initial_coordinates_set = false;

}

void Possum::odom_callback(nav_msgs::Odometry msg)
{
	//if the a co-ordinate for possum to move to has never been set before
	if (initial_coordinates_set == false){
		//check if its own co-ordinates are the same as co-ordinates relative to the world. If not,
		if((this->pose.px != msg.pose.pose.position.x) || (this->pose.py != msg.pose.pose.position.y)){
			//set possums co-ordinates so that its relative to the world.
			this->pose.px = msg.pose.pose.position.x;
			this->pose.py = msg.pose.pose.position.y;
		}
		//set the max_row variable which will indicate the total number of rows
		this->max_row = computeNumberOfRows();
		geometry_msgs::Point point;
		point.x = this->pose.px;
		point.y = this->pose.py;
		//configure the route of the possum to be travelling at 3.5 across x axis each time which is the
		//distance between each kiwi row.
		for (int i = 0; i<(this->max_row -1); i++){
			point.x = point.x + 3.5;
			action_queue.push(point);
		}
		//set this variable to true so that this whole initial setup won't occur again.
		initial_coordinates_set = true;
	}

	//update possums position (pose) based on its position relative to world.		
	this->pose.px = msg.pose.pose.position.x;
	this->pose.py = msg.pose.pose.position.y;

	//assign,update orientation(x,y,z,w) values.
	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	//print possums position and status which will be picked up and update on GUI.
	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(this->state));
	//ROS_INFO("desired_angle: %f", this->orientation.desired_angle);
	//ROS_INFO("orientation_angle: %f", this->orientation.angle);

	//ROS_INFO("goto x: %f", action_queue.front().x);
	//ROS_INFO("goto y: %f", action_queue.front().y);


	calculateOrientation(); //calculate orientation based on x,y,z,w values previously obtained.
	doAngleCheck(); //update possums current angle and desired angle
	checkTurningStatus(); //check if possum is currently facing the direction its supposed to be facing.
	//if the possums status indicates that is can move across, begin following co-ordinates in queue.
	if(this->state == MOVINGACROSS){
		begin_action2(3.0);
	}

	publish();

}

void Possum::laser_callback(sensor_msgs::LaserScan msg)
{
	//if possum is idle (is directly beneath/inside a kiwi row) and not turning
	if 	((this->state == IDLE) && (this->orientation.currently_turning == false)){
		bool can_move = true; //declare a variable which indicates whether possum can move across to next row.
		for (int i = 0; i<150; i++){
			if (msg.ranges[i] < 3){ 
				can_move = false; //if an object is detected set can_move to false
				ROS_INFO("NODE DETECTED");
				break;
			}
		}
		//if nothing has been detected by laser, change state to moveacross.
		if (can_move == true){
			this->state = MOVINGACROSS;
		}
	}
}

void Possum::stop(){
	ROS_INFO("ENTERED STOP METHOD");
	//increment or decrement 'row' depending on which direction possum i traveling at to keep track of
	//which row possum is currently at.
	if (this->direction == EAST){
		this->row = this->row +1;
	} else if (this->direction == WEST){
		this->row = this->row -1;
	}
	//if possum is at far most right row, turn it around, change its direction, and push new set of coordinates into queue.
	if (this->row == this->max_row){
		this->direction = WEST;
		geometry_msgs::Point point;
		point.x = this->pose.px;
		point.y = this->pose.py;
		for (int i = 0; i<(this->max_row-1); i++){
			point.x = point.x - 3.5;
			action_queue.push(point);
		}	
	//if possum is at far most left row, turn it around, change its direction, and push new set of coordinates into queue.
	} else if (this->row == 1) {
		this->direction = EAST;
		geometry_msgs::Point point;
		point.x = this->pose.px;
		point.y = this->pose.py;
		for (int i = 0; i<(this->max_row-1); i++){
			point.x = point.x + 3.5;
			action_queue.push(point);
		}
	} 
	//change state of possum to idle when stopped on a row.
	this->state = IDLE;
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
	//ROS_INFO("/status/%s/./", enum_to_string(this->state));
}

//make possum stop turning
void Possum::stopTurn(){
	this->orientation.currently_turning = false;
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}

//make possum walk
void Possum::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

//make possum run
void Possum::run(){
	this->speed.linear_x = 6.0;
	this->speed.angular_z = 0.0;
}

//make possum turnleft
void Possum::turnLeft(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}

//make possum turnright
void Possum::turnRight(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = -5.0;
}
//Turn back
void Possum::turnBack(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}

//override checkTurningStatus so that linear_x = 0 after turn. And change status to IDLE
void Possum::checkTurningStatus() 
{
	if(this->orientation.currently_turning == true)
	{	
		if(doubleComparator(this->orientation.angle, this->orientation.desired_angle))
		{
			ROS_INFO("Finished Turning");
			this->state = IDLE;
			this->orientation.currently_turning = false;
			this->speed.linear_x = 0.0;
			this->speed.angular_z = 0.0; 
		}
	return;
	}
}

//method which turns state into string. Used to print states in ROS_INFO.
char const* Possum::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case ROAMING:
            return "ROAMING";
        case FLEEING:
            return "FLEEING";  
        case MOVINGACROSS:
            return "MOVINGACROSS";   
        default:
            return "INVALID ENUM";
    }
 }

//computes the total number of kiwi rows in the world using possums current(starting) position.
int Possum::computeNumberOfRows(){
	if (!(int(10 * (0-this->pose.px)) % 35)){
		return int(((10 * (0-this->pose.px)) / 35)*2 +1);
	} else {
		return int(((10 * (1.75-this->pose.px)) / 35)*2); 
	}
}

//this method processes co-ordinates inside action_queue. (Derivation of original method begin_action in node.h)
void Possum::begin_action2(double speed)
{
	geometry_msgs::Point end_point = action_queue.front();
	//check if possum is at destination co-ordinate
	if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
	{
		//if true, pop co-ordinate from queue and call stop();
		ROS_INFO("xdest: %f", end_point.x);
		ROS_INFO("ydest: %f", end_point.y);
		action_queue.pop();
		stop();
	}
	//move in the x direction (towards destination)
	if(move_x(end_point.x, speed))
	{
		//move in the y direction (towards destination)
		move_y(end_point.y, speed);
	}
}	

//unimplemented methods in the node interface.
void Possum::move(){}
void Possum::collisionDetected(){} 

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "POSSUM");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Possum node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	//node.publish();
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
