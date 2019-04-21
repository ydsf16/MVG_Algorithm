// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <triangulation.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

namespace MVG {
	
void triangulate ( const Eigen::Matrix3d& K, const Eigen::Matrix4d T1, const Eigen::Matrix4d& T2, const Eigen::Vector2d& uu1, const Eigen::Vector2d& uu2, Eigen::Vector4d& X )
{
	// construct P1 P2
	const Eigen::Matrix<double, 3, 4> P1 = K * T1.block(0,0, 3, 4);
	const Eigen::Matrix<double, 3, 4> P2 = K * T2.block(0, 0, 3, 4);
	
	// get vectors
	const Eigen::Matrix<double, 1, 4>& P11 = P1.block(0, 0, 1, 4);
	const Eigen::Matrix<double, 1, 4>& P12 = P1.block(1, 0, 1, 4);
	const Eigen::Matrix<double, 1, 4>& P13 = P1.block(2, 0, 1, 4);
	
	const Eigen::Matrix<double, 1, 4>& P21 = P2.block(0, 0, 1, 4);
	const Eigen::Matrix<double, 1, 4>& P22 = P2.block(1, 0, 1, 4);
	const Eigen::Matrix<double, 1, 4>& P23 = P2.block(2, 0, 1, 4);
	
	const double& u1 = uu1[0];
	const double& v1 = uu1[1];
	const double& u2 = uu2[0];
	const double& v2 = uu2[1];
	
	// construct H matrix.
	Eigen::Matrix4d H;
	H.block(0, 0, 1, 4) = v1 * P13 - P12;
	H.block(1, 0, 1, 4) = P11 - u1 * P13;
	H.block(2, 0, 1, 4) = v2 * P23 - P22;
	H.block(3, 0, 1, 4) = P21 - u2 * P23;
	
	// SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd ( H, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix4d V = svd.matrixV();
	
	X = V.block(0, 3, 4, 1);
	X = X / X(3, 0);
} // triangulate

	
	
} // namespace MVG
