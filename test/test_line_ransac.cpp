// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <line_RANSAC.h>
#include <opencv2/opencv.hpp>

// create good data.
void createGoodData ( const double A, const double B, const double C,
                      const double minx, const double maxx,   
                      const int n,
					  const double sigma,
                      std::vector<Eigen::Vector2d>& points )
{
	// B != 0.
	points.clear();
	points.reserve(n+2);
	
	const double incx = (maxx - minx) / n;
	cv::RNG rng(cv::getTickCount());
	for(double x = minx; x < maxx; x += incx)
	{
		double y = (-C - A*x) / B;
		points.push_back(Eigen::Vector2d(x+rng.gaussian(sigma), y+rng.gaussian(sigma)));
	}
}

void createRandomData ( const double minx, const double miny,
                        const double maxx, const double maxy,
                        Eigen::Vector2d& rpt )
{
	cv::RNG rng(cv::getTickCount());
	rpt[0]  = rng.uniform(minx, maxx);
	rpt[1] = rng.uniform(miny, maxy);
}


int main ( int argc, char **argv )
{
	double A = 1.0, B = 1.0, C = 1.0;
	std::cout << "Line ground truth A B C: " << A << " " << B << " " << C <<"\n";
	
	std::vector<Eigen::Vector2d> points;
	createGoodData(A, B, C, -2.0, 2.0, 1000, 0.02, points);
	
	// add noise points
	for(int i = 0; i < 100; i ++)
	{
		Eigen::Vector2d pt;
		createRandomData(-2.0, -2.0, 2.0, 2.0, pt);
		points.push_back(pt);
	}
	
//	show points
// 	for(size_t i = 0; i < points.size(); i++)
// 	{
// 		const Eigen::Vector2d& pt = points.at(i);
// 		std::cout << std::fixed << std::setprecision(3) << pt[0] << " " << pt[1] << "\n";
// 	}
	
	std::vector<bool> is_inlier;
	int nInlier = MVG::LineRANSAC::EstimateLineRansac(points, 0.99, 0.02, A, B, C, is_inlier);
	
	std::cout << "Line estimated A B C: " << A << " " << B << " " << C <<"\n";
	std::cout << "nInlier " << nInlier <<"\n\n";
    return 0;
}
