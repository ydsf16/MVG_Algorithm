// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)


#include <line_RANSAC.h>
#include <opencv2/opencv.hpp>

namespace MVG {
	
int LineRANSAC::EstimateLineRansac ( const std::vector< Eigen::Vector2d >& points, const double p, const double sigma, double& A, double& B, double& C, std::vector<bool>& is_inlier )
{
	// Check input
	if(points.size() < 2 )
		return -1;
	
	const int n = points.size();
	int N = std::numeric_limits<int>::max();
	const int T = 0.9*n; // TODO.
	int sample_count = 0;
	const double t2 = 3.84 * sigma * sigma;
	const int s = 2;
	
	cv::RNG rng(cv::getTickCount());
	int pre_inlier_cnt = 0;
	int final_inlier_cnt = 0;
	
	std::vector<bool> tmp_is_inlier(n);
	std::fill(tmp_is_inlier.begin(), tmp_is_inlier.end(), false);
	is_inlier.reserve(n);
		
	while(sample_count < N){
		
		// Step 1. Select s(2) points and estimate a line.
		int idx1 = rng.uniform(0, n);
		int idx2 = 0;
		while(true)
		{
			idx2 = rng.uniform(0, n);
			if(idx1 != idx2)
			{
				break;
			}
		}
		
		
		const Eigen::Vector2d& p1 = points.at(idx1);
		const Eigen::Vector2d& p2 = points.at(idx2);
		
		// Estimate
		double tmpA, tmpB, tmpC;
		findLine(p1, p2, tmpA, tmpB, tmpC);
		
		// Step 2. Find inlier points.
		int inlier_cnt = 0;
		for(size_t i = 0; i < points.size(); i++)
		{
			if(getDistance(points.at(i), tmpA, tmpB, tmpC) < t2)
			{
				tmp_is_inlier.at(i) = true;
				inlier_cnt ++;
			}
			else
			{
				tmp_is_inlier.at(i) = false;
			}
		}
		
		//std::cout << idx1 << " " << idx2 << std::endl;
		// Step 3. If we have enough pionts.
		if(inlier_cnt > T)
		{
			is_inlier = tmp_is_inlier;
			A = tmpA;
			B = tmpB;
			C = tmpC;
			final_inlier_cnt = inlier_cnt;
			// Break and refine the result.
			break;
		}
		
		// Update N.
		if(inlier_cnt > pre_inlier_cnt)
		{
			pre_inlier_cnt = inlier_cnt;
			
			// recompute N
			double w = (double)inlier_cnt / (double)n; // inlier prob./ratio.
			N = log(1.0-p) / log(1.0 - std::pow(w, s));
			
			// update the final model.
			is_inlier = tmp_is_inlier;
			A = tmpA;
			B = tmpB;
			C = tmpC;
			final_inlier_cnt = inlier_cnt;
		}
		sample_count ++;
	}
	
	// TODO Refine the result.
	
	return final_inlier_cnt;
}

void LineRANSAC::findLine ( const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double& A, double& B, double& C )
{
	A = pt1[1] - pt2[1];
	B = pt2[0] - pt1[0];
	C = pt1[0]*pt2[1] - pt2[0]*pt1[1];
}

double LineRANSAC::getDistance ( const Eigen::Vector2d& pt, const double A, const double B, const double C )
{
	return std::pow((A*pt[0] + B*pt[1] + C), 2.0) / (A*A + B*B);
}
	
	
} // namespace MVG