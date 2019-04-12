
#ifndef HOMOGRAPHY_MATRIX
#define HOMOGRAPHY_MATRIX
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>
#include <vector>

namespace MVG
{
class HomographyMatrix{
public:
	HomographyMatrix();
	
	static bool findHomographyMatrix8(const std::vector<Eigen::Vector2d>& p1s, const std::vector<Eigen::Vector2d>& p2s, Eigen::Matrix3d& H );
	static bool findHomographyMatrix9(const std::vector<Eigen::Vector2d>& p1s, const std::vector<Eigen::Vector2d>& p2s, Eigen::Matrix3d& H );
	
};// HomographyMatrix

} // namespace MVG

#endif
