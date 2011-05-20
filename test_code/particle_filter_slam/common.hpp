/** Common definitions. **/

#ifndef _COMMON_
#define _COMMON_

#include <Eigen/Eigen>
#include <vector>
#include <chrono>

#define PI = 3.14159265358979

using namespace std;

typedef Eigen::Vector3f Vector3f;
typedef Eigen::Matrix3f Matrix3f;

struct Landmark
{
	Vector3f pos;
	Matrix3f cov;
	
	Landmark(float x, float y): pos(x, y, 0) {}
};

struct MeasureResult
{
	int id;
	Vector3f result;
	Matrix3f jacobian;
};

/** Generate uniform random number **/
float uniformrnd(float min = 0.0, float max = 1.0)
{
	/** Define constants for generating uniform random numbers **/
	const int A = 48271;
	const int M = 2147483647;
	const int Q = M / A;
	const int R = M % A;
	
	/** Set seeds for generating uniform random numbers **/
	int state = std::chrono::duration<int>(monotonic_clock::now()).count();
	state = A * (state % Q) - R * (state / Q);
	state = (state >= 0) ? state : (state + M);
	srand(state);
	
	return min + (max - min) * ((float) rand() / (float) RAND_MAX);
}

/** Generate normal random number **/
float normalrnd(float miu = 0.0, float gama = 1.0)
{
	float u1 = uniformrnd();
	float u2 = uniformrnd();
	u1 = (u1 == 0.0) ? 0.0000001 : u1;
	return miu + gama * sqrt(-2 * log(u1)) * cos(2 * PI * u2);
}

#endif
