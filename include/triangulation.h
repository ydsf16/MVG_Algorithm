// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <eigen3/Eigen/Core>

namespace MVG{
	
	static void triangulate(const Eigen::Matrix3d& K, 
		const Eigen::Matrix4d T1, const Eigen::Matrix4d& T2, 
		const Eigen::Vector2d& u1, const Eigen::Vector2d& u2,
		Eigen::Vector4d& X
	);
	
} // namespace MVG

#endif
