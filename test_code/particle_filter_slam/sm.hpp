/** Sensor model. **/

#ifndef _SM_
#define _SM_

#include "common.hpp"
#define DISTANCE_THRESHOLD 1.5

class SensorModel
{
  public:
	Matrix3f var; // variance
	
	/** Initialize the model variance.**/
	SensorModel() {}
	
	void setvariance(v_distance, float v_angle, float v_identity)
	{
		var << v_distance, 0, 0,
			   0, v_angle, 0,
			   0, 0, v_identity;
	}

	/** Take a real measurement (with uncertainty) **/
	bool measure(Vector3f& pos_robot, Vector3f& pos_landmark, Vector3f& result, Matrix3f& jacobian)
	{
		float distance_x = pos_landmark(0) - pos_robot(0);
		float distance_y = pos_landmark(1) - pos_robot(1);
		float distance_sq = pow(distance_x, 2) + pow(distance_y, 2);
		float distance = sqrt(distance_sq);
		result(0) = distance + normalrnd(0.0, var(0,0) * 0.25);
		if(result(0) > DISTANCE_THRESHOLD)
			return false;
		
		result(1) = atan2(distance_y, distance_x) + normalrnd(0.0, var(1,1) * 0.25);
		result(2) = 0;
		
		jacobian << - distance_x / distance, - distance_y / distance, 0.0,
					distance_y / distance_sq, distance_x / distance_sq, -1.0,
					0.0, 0.0, 1.0;
					
		return true;
	}
};

#endif
