// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)


#include <homography_matrix.h>

namespace MVG {
	
HomographyMatrix::HomographyMatrix()
{

}

bool HomographyMatrix::findHomographyMatrix8 ( const std::vector< Eigen::Vector2d >& p1s, const std::vector< Eigen::Vector2d >& p2s, Eigen::Matrix3d& H )
{
	// Check input
	if(p1s.size() < 4 || p2s.size() < 4 || p1s.size() != p2s.size())
	{
		return false;
	}
	
	// Construct M matrix.
	Eigen::MatrixXd M, b;
	M.resize(p1s.size() * 2, 8);
	b.resize(p1s.size() * 2, 1);
	
	// fill in M
	for(size_t i = 0; i < p1s.size(); i ++)
	{
		const int id = 2 * i;
		const Eigen::Vector2d& p1 = p1s.at(i);
		const Eigen::Vector2d& p2 = p2s.at(i);
		
		M(id, 0) = p1[0];
		M(id, 1) = p1[1];
		M(id, 2) = 1.0;
		M(id, 3) = 0.0;
		M(id, 4) = 0.0;
		M(id, 5) = 0.0;
		M(id, 6) = -p1[0]*p2[0];
		M(id, 7) = -p1[1]*p2[0];
		
		M(id+1, 0) = 0.0;
		M(id+1, 1) = 0.0;
		M(id+1, 2) = 0.0;
		M(id+1, 3) = p1[0];
		M(id+1, 4) = p1[1];
		M(id+1, 5) = 1.0;
		M(id+1, 6) = -p1[0]*p2[1];
		M(id+1, 7) = -p1[1]*p2[1];
		
		b(id) = p2[0];
		b(id+1) = p2[1];
	}
	
	// Solve
	Eigen::Matrix<double, 8, 1> x = M.colPivHouseholderQr().solve(b);
	
	// Assemble Homography matrix.
	H << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), 1.0;
	return true;
}

bool HomographyMatrix::findHomographyMatrix9 ( const std::vector< Eigen::Vector2d >& p1s, const std::vector< Eigen::Vector2d >& p2s, Eigen::Matrix3d& H )
{
	// Check input
	if(p1s.size() < 4 || p2s.size() < 4 || p1s.size() != p2s.size())
	{
		return false;
	}
	
	// Construct M matrix.
	Eigen::MatrixXd M;
	M.resize(p1s.size() * 2, 9);

	// fill in M
	for(size_t i = 0; i < p1s.size(); i ++)
	{
		const int id = 2 * i;
		const Eigen::Vector2d& p1 = p1s.at(i);
		const Eigen::Vector2d& p2 = p2s.at(i);
		
		M(id, 0) = p1[0];
		M(id, 1) = p1[1];
		M(id, 2) = 1.0;
		M(id, 3) = 0.0;
		M(id, 4) = 0.0;
		M(id, 5) = 0.0;
		M(id, 6) = -p1[0]*p2[0];
		M(id, 7) = -p1[1]*p2[0];
		M(id, 8) = -p2[0];
		
		M(id+1, 0) = 0.0;
		M(id+1, 1) = 0.0;
		M(id+1, 2) = 0.0;
		M(id+1, 3) = p1[0];
		M(id+1, 4) = p1[1];
		M(id+1, 5) = 1.0;
		M(id+1, 6) = -p1[0]*p2[1];
		M(id+1, 7) = -p1[1]*p2[1];
		M(id+1, 8) = -p2[1];
	}
	
	Eigen::Matrix<double, 9, 9> MTM = M.transpose() * M;
	
	Eigen::SelfAdjointEigenSolver< Eigen::Matrix<double, 9, 9> > es(MTM);
	
	Eigen::MatrixXd e_vectors = es.eigenvectors();
	
	Eigen::Matrix<double, 9, 1> x = e_vectors.block ( 0, 0, 9, 1 );
	
	// Assemble Homography matrix.
	H << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8);
	double eta = 1.0/x(8);
	H *= eta;
	
	return true;
}

	
	
} //namespace MVG

