#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

void updateGradients(float* ptrX, float* ptrY, pitch, yaw2)
{
    float totalGrad = tan(pitch);
    *ptrX = cos(yaw2)*totalGrad;
    *ptrY = sin(yaw2)*totalGrad;
    return;
}

void RANSAC(vector<pointCorrespondence> flowVectors, float SQR_ERROR, vector<float> &estTrans)
{
    int maxInliers = 0;
    vector<int> bestMatches;
    
    for(int i=0; i< pointCorrespondence.size(); i++)
    {
	int currentInliers = 0;
	vector<int> matches;
	for(int j=0; j<pointCorrespondence.size(); j++)
	{
	    if(pow(pow(flowVectors[i].distX - flowVectors[j].distX, 2) + pow(flowVectors[i].distY - flowVectors[j].distY, 2) + pow(flowVectors[i].distZ - flowVectors[j].distZ, 2),0.5) < SQR_ERROR)
	    {
		currentInliers++;
		matches.push_back(j);
	    }
	}
	if(currentInliers > maxInliers)
	{
	    maxInliers = currentInliers;
	    bestMatches = matches;
	}
    }
    

    for(int i=0; i<bestMatches.size(); i++)
    {
	estTrans[0] += flowVectors[i].distX;
	estTrans[1] += flowVectors[i].distY;
	estTrans[2] += flowVectors[i].distZ;
    }
    estTrans[0] = estTrans[0]/bestMatches.size();
    estTrans[1] = estTrans[1]/bestMatches.size();
    estTrans[2] = estTrans[2]/bestMatches.size();
}



    
void planeProjection(float yaw2, float pitch, float optDistPlane, int u, int v, float alpha_x, float alpha_y, int pixelX, int pixelY, float* egoX, float* egoY, float* egoZ)
{

    //input yaw, pitch, yaw2, and optDistPlane for each of the two frames and delta_yaw between frames
    float yaw2 = 0;
    float pitch = 0;
    float optDistPlane = 1;

    int u = 240;
    int v = 320;
    float alpha_x = 1;
    float alpha_y = 1;

    float gradX;
    float gradY;

    int pixelX;
    int pixelY;

    float imagePlaneX;
    float imagePlaneY;
    
    imagePlaneX = (pixelX-u)*alpha_x;
    imagePlaneY = (pixelY-v)*alpha_y;

    updateGradients(&gradX, &gradY, pitch, yaw2);

    if(1 - imagePlaneX*gradX - imagePlaneY*gradY <= 0)
    {
	return 0;
    }
    *egoZ = optDistPlane/(1 - imagePlaneX*gradX - imagePlaneY*gradY);
    *egoY = imagePlaneY* (*egoZ);
    *egoX = imagePlaneX* (*egoZ);
}

