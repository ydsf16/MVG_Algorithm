// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)


#include <fundamental_matrix.h>
#include <iostream>

namespace MVG {
	
	FundamentalMatrix::FundamentalMatrix()
	{
		
	}
	
	
	bool FundamentalMatrix::findFundamentalMatrix ( const std::vector< Eigen::Vector2d >& p1s, const std::vector< Eigen::Vector2d >& p2s, Eigen::Matrix3d& F )
	{
		// Check input
		if(p1s.size() < 8 || p2s.size() < 8 || p1s.size() != p2s.size())
		{
			return false;
		}
		
		// Construct M
		Eigen::MatrixXd M;
		M.resize(p1s.size(), 9);
		
		for(size_t i = 0; i < p1s.size(); i ++)
		{
			const Eigen::Vector2d& p1 = p1s.at(i);
			const Eigen::Vector2d& p2 = p2s.at(i);
			
			const double& u1 = p1(0);
			const double& v1 = p1(1);
			const double& u2 = p2(0);
			const double& v2 = p2(1);
			
			M(i, 0) = u2*u1;
			M(i, 1) = u2*v1;
			M(i, 2) = u2;
			M(i, 3) = v2*u1;
			M(i, 4) = v2*v1;
			M(i, 5) = v2;
			M(i, 6) = u1;
			M(i, 7) = v1;
			M(i, 8) = 1.0;
		}
		
		// Solve 
		Eigen::Matrix<double, 9, 9> MTM = M.transpose() * M;
		Eigen::SelfAdjointEigenSolver< Eigen::Matrix<double, 9, 9> > es(MTM);
		Eigen::MatrixXd e_vectors = es.eigenvectors();
		Eigen::Matrix<double, 9, 1> x = e_vectors.block ( 0, 0, 9, 1 );
		
		// Assemble Homography matrix.
		F << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8);
		
		Eigen::JacobiSVD<Eigen::MatrixXd> svd ( F, Eigen::ComputeFullU | Eigen::ComputeFullV );
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Vector3d sig_val = svd.singularValues();
		
		Eigen::Matrix3d Sigma;
		Sigma << sig_val(0), 0.0, 0.0, 
		0.0, sig_val(1), 0.0,
		0.0, 0.0, 0.0;
		
		F = U * Sigma * V.transpose();
		
		F = F / F(2,2);
		
		return true;
	} // findFundamentalMatrix
	
	
} // namespace MVG