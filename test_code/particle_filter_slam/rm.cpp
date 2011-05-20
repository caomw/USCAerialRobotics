/** Robot model. **/

#ifndef _RM_
#define _RM_

#include "common.hpp"

class RobotModel
{
  public:
	Vector3f pos;
	Vector3f var;
	
  	RobotModel() {}
  	
  	void setvariance(float v_speed, float v_rotation)
  	{
		var << v_speed, v_rotation, 0;
	}
	
	void setposition(float x, float y, float theta)
	{
		pos << x, y, theta;
	}
	
	/** Move the robot (command + uncertainty) **/
	Vector3f move(float speed, float rotation)
	{
		speed = normalrnd(speed(1), var(0)*.25);
		rotation = normalrnd(rotation(2), var(1)*.25);
		
		Vector3f result;
		result(0) = cos(pos(2) + rotation) * speed;
		resutl(1) = sin(pos(2) + rotation) * speed;
		result(2) = rotation;
		pos += result;
		return pos;
	}
};

#endif
