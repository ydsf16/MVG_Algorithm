
#ifndef FUNDAMENTAL_MATRIX
#define FUNDAMENTAL_MATRIX

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

namespace MVG{
	
	class FundamentalMatrix{
	public:
		FundamentalMatrix();
		
		static bool findFundamentalMatrix(const std::vector<Eigen::Vector2d>& p1s, const std::vector<Eigen::Vector2d>& p2s, Eigen::Matrix3d& F);
		
		
	}; //class FundamentalMatrix
	
} // namespace MVG

#endif
