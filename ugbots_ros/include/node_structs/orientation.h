class Orientation
{
public:
	int previous_right_distance;
	int previous_left_distance;
	int previous_front_distance;
	bool currently_turning;
	bool currently_turning_static;

	double rotx;
	double roty;
	double rotz;
	double rotw;
	double angle;
	double desired_angle;
};
