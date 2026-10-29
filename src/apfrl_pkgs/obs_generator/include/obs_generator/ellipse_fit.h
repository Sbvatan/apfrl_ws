#ifndef ELLIPSE_FIT_H
#define ELLIPSE_FIT_H

#include <complex>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <iostream>
#include <vector>
#include <cmath>



#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


//#include <obs_generator/data_structures.h>
#include "data_structures.h"

/// @brief Ellipse Fit using Least Squares Fitting of Ellipses
class EllipseFit
{
public:
    /// @brief Fits ellipse to input points using Least Squares Fitting of Ellipses
    /// @param input Points
    /// @param output Output parameters of the ellipse
    static void fit(const std::vector<Point> &input, Obstacle &output);

    static void fitM(const std::vector<Point> &input, Obstacle &output);


    void static decompose(Eigen::MatrixXd&,
                   Eigen::VectorXd&,
                   Eigen::MatrixXd&,
                   Eigen::MatrixXd&);

    // static std::vector<double> centroid();
    // static std::vector<double> radii();
    // static std::vector<std::vector<double>> pose();

    //void compute(std::vector<std::vector<double>>&, double, double);
    Obstacle static compute(Eigen::MatrixXd&, double, double);

    Obstacle static compute(const std::vector<Point>  data, double eps, double lmcoeff);

    Obstacle static computeAlperen(const std::vector<Point>  data, double eps);

    // Data members
    long int static iters;
    static std::vector<double>   _centroid;
    static std::vector<double> _radii;
    static std::vector<std::vector<double>> _pose;



private:


    void static khachiyan(Eigen::MatrixXd& data, double eps, double lmcoeff,
                          std::vector<double>*   _centroid,
                          std::vector<double>* _radii,
                          std::vector<std::vector<double>>* _pose);

};
#endif
