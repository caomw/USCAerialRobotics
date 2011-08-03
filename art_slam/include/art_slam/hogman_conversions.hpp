#ifndef _ART_SLAM_HOGMAN_CONVERSIONS_
#define _ART_SLAM_HOGMAN_CONVERSIONS_

Transformation3 cvt_eigen_to_hogman(Eigen::Matrix4f& trafo)
{
	Eigen::Affine3f trafo_affine(trafo);
	Transformation3 result(Vector3(trafo(0, 3), trafo(1, 3), trafo(2, 3)), Quaternion(trafo_affine.rotation().x(), trafo_affine.rotation().y(), trafo_affine.rotation().z(), eigen_quat.w()));
	return result;
}

#endif
