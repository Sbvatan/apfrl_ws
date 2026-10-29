//#include <obs_generator/ellipse_fit.h>
#include "obs_generator/ellipse_fit.h"
#include "boost/chrono.hpp"
#include "obs_generator/utils.hpp"
#include <chrono>
using namespace Eigen;

namespace {

// A conic fit is only meaningful when the cluster actually has two distinguishable
// axes. When it does not, the closed form below divides by ~0 and the ellipse comes
// back NaN, infinite, or vanishingly small - and an obstacle that vanishes is worse
// than a crude one, because the planner then drives through it.
//
// Both degenerate cases are ordinary laser geometry, not corner cases:
//   * collinear points  (b^2 - a*c -> 0)  - a flat wall, most of an indoor scan
//   * a circular cluster (a - c   -> 0)  - a pillar, a table leg, a PERSON
//
// Measured in the AWS small house, 21328 ellipses over 75 s produced zero
// non-finite values, so this path is not hot there. That house is flat walls and
// boxy furniture; the a == c branch needs a round cluster, which is what a
// pedestrian is. The guard is here before the actor worlds rather than after.

// Centroid and enclosing radius. Not the minimum enclosing circle - this is O(n),
// never smaller than the true extent, and erring large is the safe direction for
// an obstacle.
void boundingCircle(const std::vector<Point> &pts, Obstacle &out)
{
  out.x = out.y = out.r_a = out.r_b = out.yaw = 0.0;
  if (pts.empty()) {
    return;
  }
  double cx = 0.0;
  double cy = 0.0;
  for (const auto &pt : pts) {
    cx += pt.x;
    cy += pt.y;
  }
  cx /= static_cast<double>(pts.size());
  cy /= static_cast<double>(pts.size());

  double r = 0.0;
  for (const auto &pt : pts) {
    r = std::max(r, std::hypot(pt.x - cx, pt.y - cy));
  }
  out.x = cx;
  out.y = cy;
  out.r_a = r;
  out.r_b = r;
  out.yaw = 0.0;
}

// Conic coefficients -> ellipse. Returns false and leaves `out` alone when the
// conic is degenerate or the result is not finite; the caller substitutes a
// bounding circle. When the fit is well conditioned this is the original
// arithmetic unchanged, so a healthy scan produces exactly what it did before.
bool ellipseFromConic(double a, double b, double c, double d, double f, double g,
                      Obstacle &out)
{
  constexpr double kEps = 1e-12;

  const double disc = b * b - a * c;
  if (!std::isfinite(disc) || std::abs(disc) < kEps) {
    return false;                       // collinear: no unique centre
  }
  const double ac = a - c;
  if (!std::isfinite(ac) || std::abs(ac) < kEps) {
    return false;                       // circular: a circle is the honest answer
  }

  const double centre_x = (c * d - b * f) / disc;
  const double centre_y = (a * f - b * d) / disc;

  const double numerator = 2 * (a * f * f + c * d * d + g * b * b - 2 * b * d * f - a * c * g);
  const double root = std::sqrt(1 + 4 * b * b / (ac * ac));
  const double denominator1 = disc * ((c - a) * root - (c + a));
  const double denominator2 = disc * ((a - c) * root - (c + a));
  if (std::abs(denominator1) < kEps || std::abs(denominator2) < kEps) {
    return false;
  }

  const double width_sq = numerator / denominator1;
  const double height_sq = numerator / denominator2;
  if (!(width_sq > 0.0) || !(height_sq > 0.0)) {
    return false;                       // sqrt of a negative: not an ellipse
  }

  const double width = std::sqrt(width_sq);
  const double height = std::sqrt(height_sq);
  const double phi = 0.5 * std::atan((2 * b) / ac);
  if (!std::isfinite(centre_x) || !std::isfinite(centre_y) ||
      !std::isfinite(width) || !std::isfinite(height) || !std::isfinite(phi)) {
    return false;
  }

  out.x = centre_x;
  out.y = centre_y;
  out.r_a = width;
  out.r_b = height;
  out.yaw = phi;
  return true;
}

}  // namespace





void EllipseFit::fit(const std::vector<Point> &input, Obstacle &output)
{

	size_t size_mat = input.size();
	auto X_val = Eigen::VectorXd(size_mat);
	auto Y_val = Eigen::VectorXd(size_mat);
	for (unsigned int i = 0U; i < size_mat; ++i)
	{
        X_val[i] = input[i].x;  Y_val[i] = input[i].y;
	}

	Eigen::VectorXd D1_col0(size_mat);
	Eigen::VectorXd D1_col1(size_mat);
	Eigen::VectorXd D1_col2(size_mat);

	D1_col0 = X_val.array().pow(2);
	D1_col1 = X_val.array() * Y_val.array();
	D1_col2 = Y_val.array().pow(2);

	Eigen::MatrixXd D1(size_mat, 3);

	D1.col(0) = D1_col0;
	D1.col(1) = D1_col1;
	D1.col(2) = D1_col2;

	Eigen::MatrixXd D2(size_mat, 3);

	D2.col(0) = X_val;
	D2.col(1) = Y_val;
	D2.col(2) = Eigen::VectorXd::Ones(size_mat);

	Eigen::MatrixXd S1(3, 3);
	Eigen::MatrixXd S2(3, 3);
	Eigen::MatrixXd S3(3, 3);

	S1 = D1.transpose() * D1;
	S2 = D1.transpose() * D2;
	S3 = D2.transpose() * D2;

	Eigen::MatrixXd C1(3, 3);
	C1 << 0, 0, 2, 0, -1, 0, 2, 0, 0;

	Eigen::MatrixXd M;
	M = C1.inverse() * (S1 - S2 * S3.inverse() * S2.transpose());
	Eigen::EigenSolver<MatrixXd> s(M);
	std::complex<double> zero(0, 0);
	MatrixXd eigenvector = s.eigenvectors().real();
	MatrixXd eigenvectorimag = s.eigenvectors().imag();
	bool iscomplex{false};
	for (int i = 0; i < 3; i++)
	{
		if (iscomplex)
			break;
		for (int j = 0; j < 3; j++)
		{
			if (eigenvectorimag(i, j) != zero.imag())
			{
				iscomplex = true;
				break;
			}
		}
	}
	if (!iscomplex)
	{
		Eigen::VectorXd eig_row0 = eigenvector.row(0);
		Eigen::VectorXd eig_row1 = eigenvector.row(1);
		Eigen::VectorXd eig_row2 = eigenvector.row(2);

		Eigen::VectorXd cond = 4 * (eig_row0.array() * eig_row2.array()) - eig_row1.array().pow(2);

		Eigen::VectorXd min_pos_eig;

		for (int i = 0; i < 3; i++)
		{
			if (cond(i) > 0)
			{
				min_pos_eig = eigenvector.col(i);
				break;
			}
		}
		Eigen::VectorXd coeffs(6);
		if (min_pos_eig.size() > 0)
		{
			Eigen::VectorXd cont_matrix = -1 * S3.inverse() * S2.transpose() * min_pos_eig;
			coeffs << min_pos_eig, cont_matrix;
			double a = coeffs(0);
			double b = coeffs(1) / 2;
			double c = coeffs(2);
			double d = coeffs(3) / 2;
			double f = coeffs(4) / 2;
			double g = coeffs(5);

			if (!ellipseFromConic(a, b, c, d, f, g, output)) {
				boundingCircle(input, output);
			}
		}
		else
		{
			boundingCircle(input, output);
		}
	}
	else
	{
		boundingCircle(input, output);
	}
}



void EllipseFit::fitM(const std::vector<Point> &input, Obstacle &output)
{

    size_t size_mat = input.size();
    auto X_val = Eigen::VectorXd(size_mat);
    auto Y_val = Eigen::VectorXd(size_mat);
    for (unsigned int i = 0U; i < size_mat; ++i)
    {
        X_val[i] = input[i].x;
        Y_val[i] = input[i].y;
    }

    Eigen::VectorXd D1_col0(size_mat);
    Eigen::VectorXd D1_col1(size_mat);
    Eigen::VectorXd D1_col2(size_mat);

    D1_col0 = X_val.array().pow(2);
    D1_col1 = X_val.array() * Y_val.array();
    D1_col2 = Y_val.array().pow(2);

    Eigen::MatrixXd D1(size_mat, 3);

    D1.col(0) = D1_col0;
    D1.col(1) = D1_col1;
    D1.col(2) = D1_col2;

    Eigen::MatrixXd D2(size_mat, 3);

    D2.col(0) = X_val;
    D2.col(1) = Y_val;
    D2.col(2) = Eigen::VectorXd::Ones(size_mat);

    Eigen::MatrixXd S1(3, 3);
    Eigen::MatrixXd S2(3, 3);
    Eigen::MatrixXd S3(3, 3);

    S1 = D1.transpose() * D1;
    S2 = D1.transpose() * D2;
    S3 = D2.transpose() * D2;

    Eigen::MatrixXd C1(3, 3);
    C1 << 0, 0, 2, 0, -1, 0, 2, 0, 0;

    Eigen::MatrixXd M;
    M = C1.inverse() * (S1 - S2 * S3.inverse() * S2.transpose());
    Eigen::EigenSolver<MatrixXd> s(M);
    std::complex<double> zero(0, 0);
    MatrixXd eigenvector = s.eigenvectors().real();
    MatrixXd eigenvectorimag = s.eigenvectors().imag();



    Eigen::VectorXd eig_row0 = eigenvector.row(0);
    Eigen::VectorXd eig_row1 = eigenvector.row(1);
    Eigen::VectorXd eig_row2 = eigenvector.row(2);

    Eigen::VectorXd cond = 4 * (eig_row0.array() * eig_row2.array()) - eig_row1.array().pow(2);

    Eigen::VectorXd min_pos_eig;

    for (int i = 0; i < 3; i++)
    {
        if (cond(i) > 0)
        {
            min_pos_eig = eigenvector.col(i);
            break;
        }
    }

    Eigen::VectorXd coeffs(6);
    if (min_pos_eig.size() > 0)
    {
        Eigen::VectorXd cont_matrix = -1 * S3.inverse() * S2.transpose() * min_pos_eig;
        coeffs << min_pos_eig, cont_matrix;
        double a = coeffs(0);
        double b = coeffs(1) / 2;
        double c = coeffs(2);
        double d = coeffs(3) / 2;
        double f = coeffs(4) / 2;
        double g = coeffs(5);

        if (!ellipseFromConic(a, b, c, d, f, g, output)) {
            boundingCircle(input, output);
        }
    }
    else
    {
        boundingCircle(input, output);
    }


}





void EllipseFit::decompose(MatrixXd& A, VectorXd& s, MatrixXd& U, MatrixXd& V)
{
    JacobiSVD<MatrixXd> SVD(A, ComputeThinU | ComputeThinV);
    s = SVD.singularValues();
    V = SVD.matrixV();
    U = SVD.matrixU();
}


/*
    Khachiyan's ellipsoid method
    @param X: matrix of data (n x d)
    @param eps: approximation error (default: 0.001)
    @param lmcoeff: Levenberg-Marquardt coefficient
                    used for inverting ill-condditioned
                    matrices. (default: 1e-8)
    Ref: Khachiyan (1979)
*/
void EllipseFit::khachiyan(MatrixXd& data, double eps, double lmcoeff,
                           std::vector<double>* _centroid,
                           std::vector<double>* _radii,
                           std::vector<std::vector<double>>* _pose)
{
    // Khachiyan's algorithm
    auto t0 = std::chrono::system_clock::now();

    long int  iters = 0;



    long double err = INFINITY;
    double alpha;
    int n = data.rows();
    int d = data.cols();
    MatrixXd X = data.transpose();
    MatrixXd Q(X.rows() + 1, X.cols());
    Q.row(0) = X.row(0);
    Q.row(1) = X.row(1);
    Q.row(2).setOnes();
    VectorXd u = (1 / (double) n) * VectorXd::Ones(n);
    VectorXd uhat = u;
    MatrixXd G(d + 1, d + 1);
    MatrixXd noiseye = MatrixXd::Identity(G.rows(), G.cols()) * lmcoeff;
    VectorXd g(n);
    double m; int i;
    while (err > eps)
    {
        G = Q * u.asDiagonal() * Q.transpose() + noiseye;
        g = (Q.transpose() * G.inverse() * Q).diagonal();
        m = g.maxCoeff();
        i = findIdx(g, m);
        alpha = (m - d - 1) / ((d + 1) * (m - 1));
        uhat = (1 - alpha) * u;
        uhat(i) += alpha;
        err = (uhat - u).norm();
        u = uhat;
        iters++;
    }
    auto tf = std::chrono::duration<double>(
              std::chrono::system_clock::now() - t0
               ).count();

    // Decompose US^2U^T
    MatrixXd E = (1 / (double) d) * (
                                        X * u.asDiagonal() * X.transpose()
                                        - (X * u) * (X * u).transpose()
                                        ).inverse();
    VectorXd x = X * u;
    VectorXd s(d);
    MatrixXd V(d, d);
    MatrixXd U(d, d);
    decompose(E, s, U, V);
    std::vector<double> centroid;
    std::vector<double> radii;
    std::vector<std::vector<double>> pose;
    // Cache result
    centroid = toStdVec(x, [](double z) { return z; });
    radii    = toStdVec(s, [](double z) { return 1 / sqrt(z); });
    pose     = toStdMat(V, [](double z) { return z; });

    *_centroid = centroid;
    *_radii = radii;
    *_pose = pose;

}






/*
    Computes minimum volume ellipse enclosing data
    @param data:vector<vector<double>> data (n x d)
    @param delim:char delimiter character
    @param eps:double approximation error (default: 0.001)
*/
Obstacle EllipseFit::compute(const std::vector<Point>  data, double eps, double lmcoeff)
{
    int rows = data.size();
    int cols = 2;
    MatrixXd D(rows, cols);
    for (int i=0; i<rows; i++)
    {
        D(i,0) = data[i].x;  D(i,1) = data[i].y;

    }
    return EllipseFit::compute(D, eps,lmcoeff);
}

Obstacle EllipseFit::computeAlperen(const std::vector<Point>  data, double eps)
{
    Obstacle obs{};
    int rows = data.size();
    int cols = 2;
    obs.r_a = sqrt(pow(data[0].x-data[rows-1].x,2) + pow(data[0].y-data[rows-1].y,2))/2;
    obs.r_b = eps;
    obs.yaw = atan2(data[rows-1].y-data[0].y,data[rows-1].x-data[0].x);
    obs.x = (data[0].x + data[rows-1].x)/2;
    obs.y = (data[0].y + data[rows-1].y)/2;


    /*
    std::cout << "Point1: " << data[0].x << " " << data[0].y <<" Point2: " << data[rows-1].x << " " << data[rows-1].y <<
        " X: " << obs.x << " Y: " << obs.y << " r_a: " << obs.r_a << " r_b: " << obs.r_b << " yaw: " << obs.yaw << std::endl;
    */
    return obs;
}

/*
    Computes minimum volume ellipse enclosing data in file
    @param file:Eigen::MatrixXd data (n x d)
    @param delim:char delimiter character
    @param eps:double approximation error (default: 0.001)
*/
Obstacle EllipseFit::compute(MatrixXd& data, double eps, double lmcoeff)
{
    std::vector<double>   centroid;
    std::vector<double> radii;
    std::vector<std::vector<double>> pose;

    khachiyan(data, eps, lmcoeff, &centroid, &radii, &pose);

    Obstacle obs;

    obs.x = centroid[0]; obs.y=centroid[1];
    obs.r_a = radii[0]; obs.r_b = radii[1];
    obs.yaw = atan2(pose[1][0], pose[0][0]);
    return obs;
}


// /*
//     Ellipse centroid
//     @return centroid:vector<double> (d x 1)
//     Note: Invoke compute() prior to retrieving
//             the ellipse parameters.
// */
// vector<double> EllipseFit::centroid()
// {
//     return _centroid;
// }


// /*
//     Ellipse radii
//     @return radii:vector<double> (d x 1)
//     Note: Invoke compute() prior to retrieving
//             the ellipse parameters.
// */
// vector<double> EllipseFit::radii()
// {
//     return _radii;
// }


// /*
//     Ellipse pose (rotation)
//     @return pose:vector<vector<double>> (d x d)
//     Note: Invoke compute() prior to retrieving
//             the ellipse parameters.
// */
// vector<vector<double>> EllipseFit::pose()
// {
//     return _pose;
// }


