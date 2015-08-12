class Speed
{
public:
	//Setting the linear x. Must be within max_linear_x or half that if going in reverse
	int setLinearX(double x)
	{
		if ((x > max_linear_x) || (x < -0.5*max_linear_x))
		{
			return 1;
		}
		linear_x = x;
		return 0;
	}

	int setAngularZ(double z)
	{
		angular_z = z;
		return 0;
	}

	int setMaxSpeed(double x)
	{
		max_linear_x = x;
		return 0; 
	}

	double getLinearX()
	{
		return  linear_x;
	}

	double getAngularZ()
	{
		return angular_z;
	}

	double getMaxSpeed()
	{
		return max_linear_x;
	}

private:
	double linear_x;
	double angular_z;
	double max_linear_x;
	double max_angular_z; 	//Implement max turning speed
};