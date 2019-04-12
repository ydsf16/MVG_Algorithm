
#ifndef ESSENTIAL_MATRIX
#define ESSENTIAL_MATRIX

#include <fundamental_matrix.h>

namespace MVG{
	
class EssentialMatrix{
public:
	EssentialMatrix();
	
	static bool findEssentialMatrix(const std::vector<Eigen::Vector2d>& p1s, const std::vector<Eigen::Vector2d>& p2s, const Eigen::Matrix3d& K, Eigen::Matrix3d& E);
	
}; // class EssentialMatrix
	
} // namespace MVG

#endif