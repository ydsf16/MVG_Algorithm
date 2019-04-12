// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)


#include <essential_matrix.h>

namespace MVG {
	
EssentialMatrix::EssentialMatrix()
{

}

bool EssentialMatrix::findEssentialMatrix ( const std::vector< Eigen::Vector2d >& p1s, const std::vector< Eigen::Vector2d >& p2s, const Eigen::Matrix3d& K, Eigen::Matrix3d& E )
{
	Eigen::Matrix3d F;
	bool sysok = FundamentalMatrix::findFundamentalMatrix(p1s, p2s, F);
	if(!sysok)
	{
		return false;
	}
	
	E = K.transpose() * F * K;
	
	return true;
} // findEssentialMatrix

	
	
} // namespace MVG