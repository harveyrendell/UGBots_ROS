/**
	Author: UGBots

	The objects of this class linger/run around the outer fence of the orchard.
	When obstacles are detected in front, they turn around and travel the opposite direction.
	Their states regularly change between IDLE, WALKING, RUNNING which reflects their linear
	and angular speed.
*/

#include <node_defs/cat.h>

Cat::Cat()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	//setup initial states
	this->state = IDLE;
	this->direction = CLOCKWISE;
	this->position = NORTH;
}

Cat::Cat(ros::NodeHandle &n)
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	//setup initial states
	this->state = IDLE;
	this->direction = CLOCKWISE;
	this->position = NORTH;

	//register with neccessary topics
	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Cat::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Cat::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Cat::timerCallback, this);

	//push first co-ordinate the cat will travel to onto queue.
	//initial destination will be top right corner.
	geometry_msgs::Point point;
	point.x = 47.0;
	point.y = 47.0;
	action_queue.push(point);
}

void Cat::odom_callback(nav_msgs::Odometry msg)
{
	//update cats position (pose) based on its position relative to world.	
	this->pose.px = msg.pose.pose.position.x;
	this->pose.py = msg.pose.pose.position.y;

	//assign,update orientation(x,y,z,w) values.
	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	//print cats position and status which will be picked up and update on GUI.
	ROS_INFO("/position/x/%f", this->pose.px); 
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(this->state));

	calculateOrientation(); //calculate orientation based on x,y,z,w values previously obtained.
	if(this->orientation.currently_turning == false){
		//if cat isn't currently turning, head towards the co-ordinate on top of queue.
		begin_action(this->speed.linear_x);
	}
	doAngleCheck(); //update cats current angle and desired angle
	checkTurningStatus(); //check if cat is currently facing the direction its supposed to be facing.
	publish(); 

}

void Cat::laser_callback(sensor_msgs::LaserScan msg)
{
	//Use laser to detect for objects when cat isn't turning.
	if(this->orientation.currently_turning == false){
		//if there is something within 2.5 range, make cat turn the other way.
		for(int i=0; i<30; i++){
			if(msg.ranges[i] < 2.5){
				ROS_INFO("/message/%s/./", "LASER DETECTED!");
				this->speed.linear_x = 0.0; //set linear to 0 and publish straight away for quick reflexes of cat.
				publish();
				turnBack();
				break;
			}
		}
	}
}

//method that is called by ROS timer every 5 seconds
void Cat::timerCallback(const ros::TimerEvent& e){
	//update status of cat if it isn't turning.
	if (this->orientation.currently_turning == false){
		setStatus();
	}
}

//method which alters cats Speed depending on its state.
void Cat::setStatus(){
	state = generateStatus();
	if (this->state == IDLE){
		stop();
	}else if (this->state == ROAMING){
		walk();
	}else{
		run();
	}
}

//method called after cat reaches a destination from the action_queue and the co-ordinate(destination) is popped from queue.
void Cat::stopAfterPop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;
	geometry_msgs::Point point;

	//Update the cats new position and depending on which direction the cat is travelling at and where it currently is, 
	//setup the next co-ordinate it should travel to and push onto the queue. 
	if (this->direction == CLOCKWISE){
		if (this->position == NORTH){
			this->position = EAST;
			point.x = 47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == EAST){
			this->position = SOUTH;
			point.x = -47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == SOUTH){
			this->position = WEST;
			point.x = -47.0;
			point.y = 47.0;
			action_queue.push(point);
		}else{
			this->position = NORTH;
			point.x = 47.0;
			point.y = 47.0;
			action_queue.push(point);
		}

	} else {
		if (this->position == NORTH){
			this->position = WEST;
			point.x = -47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == WEST){
			this->position = SOUTH;
			point.x = 47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == SOUTH){
			this->position = EAST;
			point.x = 47.0;
			point.y = 47.0;
			action_queue.push(point);
		}else{
			this->position = NORTH;
			point.x = -47.0;
			point.y = 47.0;
			action_queue.push(point);
		}
	}
}

//Stops the cat.
void Cat::stop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}

//Makes cat walk
void Cat::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

//Makes cat run
void Cat::run(){
	this->speed.linear_x = 5.0;
	this->speed.angular_z = 0.0;
}

//Makes cat turnaround.
void Cat::turnBack(){
	//empty action queue
	while (!action_queue.empty()){
		action_queue.pop();
	}

	turn(M_PI, 0.0, (M_PI/2)); //set cats desired angle to 180deg more than it is now.

	//change direction of cat
	if (this->direction == CLOCKWISE){
		this->direction = ANTICLOCKWISE;
	} else {
		this->direction = CLOCKWISE;
	}

	//Depending on which direction the cat is travelling at and where it currently is, 
	//setup the next co-ordinate it should travel to and push onto the queue. 
	geometry_msgs::Point point;
	if (this->direction == CLOCKWISE){
		if (this->position == NORTH){
			point.x = 47.0;
			point.y = 47.0;
			action_queue.push(point);
		}else if(this->position == EAST){
			point.x = 47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == SOUTH){
			point.x = -47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else{
			point.x = -47.0;
			point.y = 47.0;
			action_queue.push(point);
		}
	} else {
		if (this->position == NORTH){
			point.x = -47.0;
			point.y = 47.0;
			action_queue.push(point);
		}else if(this->position == WEST){
			point.x = -47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else if(this->position == SOUTH){
			point.x = 47.0;
			point.y = -47.0;
			action_queue.push(point);
		}else{
			point.x = 47.0;
			point.y = 47.0;
			action_queue.push(point);
		}
	}

}

//override checkTurningStatus() inorder to update cats status everytime after turning so that it
//has a new speed to travel at.
void Cat::checkTurningStatus(){
	if(this->orientation.currently_turning == true)
	{	
		if(doubleComparator(orientation.angle, orientation.desired_angle))
		{
			this->orientation.currently_turning = false;
			setStatus();
		}
	return;
	}
}

//method which turns state into string. Used to print states in ROS_INFO.
char* Cat::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case ROAMING:
            return "ROAMING";
        case RUNNING:
            return "RUNNING";  
        default:
            return "INVALID ENUM";
    }
 }

//Returns a cats status randomly.
Cat::State Cat::generateStatus(){
	//Generate a random number and return a corresponding status.
	//Running has been assigned 2 values to inc. chance of cat running.
	int randNum;
	srand (time(NULL));
	randNum = rand() % 4 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return ROAMING;
	} else {
		return RUNNING;
	}
}

//override method to make it call StopAfterPop() instead of Stop() after reaching its destination.
//Reason for not simply overriding Stop() instead is because its called by other methods in node.h
bool Cat::begin_action(double speed){

	if (action_queue.empty())
	{
		return true;
	}
	geometry_msgs::Point end_point = action_queue.front();
	if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
	{
		action_queue.pop();
		stopAfterPop();
		return true;
	}

	if(move_x(end_point.x, speed))
	{
		if(move_y(end_point.y, speed))
		{
		}

	}
}	

//unimplemented methods in the node interface.
void Cat::move(){}
void Cat::collisionDetected(){};

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CAT");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Cat node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}