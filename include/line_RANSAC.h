
#ifndef LINE_RANSAC_H
#define LINE_RANSAC_H

#include <vector>
#include <eigen3/Eigen/Core>

namespace MVG{
	
class LineRANSAC{
public: 
	static int EstimateLineRansac(const std::vector<Eigen::Vector2d>& points,
		const double p, const double sigma,
		double& A, double& B, double& C, std::vector<bool>& is_inlier 
	);
	
	static void findLine(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double& A, double& B, double& C);
	
	static double getDistance(const Eigen::Vector2d& pt, const double A, const double B, const double C);
		
	
} ;// class RANSAC
	
} //namespace MVG

#endif
