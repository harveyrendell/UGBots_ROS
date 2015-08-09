class Speed
{
public:
	void setSpeed(double x, double z)
	{
		linear_x = x;
		angular_z = z;
	}

	double getLinearVel()
	{
		return this->linear_x;
	}

	double getAngularVel()
	{
		return this->angular_z;
	}

	double getMaxVel()
	{
		return max_linear_x;
	}
//private:
	double linear_x;
	double angular_z;
	double max_linear_x;
};