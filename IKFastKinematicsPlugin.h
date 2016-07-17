#ifndef IKFASTKINEMATICSPLUGIN_H
#define IKFASTKINEMATICSPLUGIN_H

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include "lwr_ikfast.cpp"
#include "ikfast.h"
#include <iostream>

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif

using namespace ikfast;

class IKFastKinematicsPlugin
{
    public:

      std::vector<double> free_params_;
      void ComputeIkKukaLWR(Eigen::Affine3d &pose, std::vector<std::vector<double> > &ik_solutions, std::vector<double> &solution);

    private:

      void fillFreeParams(int count, int *array);
      int solve(Eigen::Affine3d &pose, const std::vector<double> &vfree, IkSolutionList<IkReal> &solutions) const;
      void getSolution(const IkSolutionList<IkReal> &solutions, int i, std::vector<double>& solution) const;
      bool getCount(int &count, const int &max_count, const int &min_count) const;
};

#endif