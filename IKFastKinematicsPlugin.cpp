#include "IKFastKinematicsPlugin.h"

void IKFastKinematicsPlugin::fillFreeParams(int count, int *array)
{
  free_params_.clear();
  for(int i=0; i<count;++i) free_params_.push_back(array[i]);
}

int IKFastKinematicsPlugin::solve(Eigen::Affine3d &pose, const std::vector<double> &vfree, IkSolutionList<IkReal> &solutions) const
{
  IKREAL_TYPE eerot[9],eetrans[3];

  // IKFast56/61
  solutions.Clear();

  eetrans[0] = pose.translation().x();
  eetrans[1] = pose.translation().y();
  eetrans[2] = pose.translation().z();

  double qw = Eigen::Quaterniond(pose.rotation()).w();
  double qx = Eigen::Quaterniond(pose.rotation()).x();
  double qy = Eigen::Quaterniond(pose.rotation()).y();
  double qz = Eigen::Quaterniond(pose.rotation()).z();

  //Converting the quaternion to union quaternion
  const double n = 1.0f/sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

  qw *= n;
  qx *= n;
  qy *= n;
  qz *= n;

  //Calculating the rotation matrix
  eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
  eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
  eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

  //Computing the ik solution
  ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  return solutions.GetNumSolutions();
}

void IKFastKinematicsPlugin::getSolution(const IkSolutionList<IkReal> &solutions, int i, std::vector<double>& solution) const
{
  solution.clear();
  solution.resize(GetNumJoints());

  // IKFast56/61
  const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  std::vector<IkReal> vsolfree( sol.GetFree().size() );
  sol.GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
}

//if both ik_solutions and best_solution are zero, then there is no solution for the ik
void IKFastKinematicsPlugin::ComputeIkKukaLWR(Eigen::Affine3d &pose, std::vector<std::vector<double> > &ik_solutions, std::vector<double> &solution)
{
  std::vector<double> ik_seed_state;
  // std::vector<double> consistency_limits;
  // std::vector<double> joint_max_vector_;

  unsigned int num_of_joints = GetNumJoints();
  unsigned int num_free_parameters = GetNumFreeParameters();

  ik_seed_state.resize(num_of_joints, 0.0);
  fillFreeParams( num_free_parameters, GetFreeParameters() );

  int counter = 0;

  std::vector<double> vfree(free_params_.size());
  double initial_guess = ik_seed_state[free_params_[0]];
  vfree[0] = initial_guess;

  // -------------------------------------------------------------------------------------------------
  // Handle consitency limits if needed
  int num_positive_increments;
  int num_negative_increments;
  double search_discretization_ = 0.005;
  std::vector<double> joint_min_vector_;
  std::vector<double> joint_max_vector_;
  std::vector<bool> joint_has_limits_vector_;

  for (int i = 0; i < num_of_joints; ++i)
  {
    joint_has_limits_vector_.push_back(1);
  }

  joint_min_vector_.push_back(-2.96706);
  joint_min_vector_.push_back(-2.0944);
  joint_min_vector_.push_back(-2.96706);
  joint_min_vector_.push_back(-2.0944);
  joint_min_vector_.push_back(-2.96706);
  joint_min_vector_.push_back(-2.0944);
  joint_min_vector_.push_back(-2.96706);

  joint_max_vector_.push_back(2.96706);
  joint_max_vector_.push_back(2.0944);
  joint_max_vector_.push_back(2.96706);
  joint_max_vector_.push_back(2.0944);
  joint_max_vector_.push_back(2.96706);
  joint_max_vector_.push_back(2.0944);
  joint_max_vector_.push_back(2.96706);

  // if(!consistency_limits.empty())
  // {
  //   // moveit replaced consistency_limit (scalar) w/ consistency_limits (vector)
  //   // Assume [0]th free_params element for now.  Probably wrong.
  //   double max_limit = fmin(joint_max_vector_[free_params_[0]], initial_guess+consistency_limits[free_params_[0]]);
  //   double min_limit = fmax(joint_min_vector_[free_params_[0]], initial_guess-consistency_limits[free_params_[0]]);

  //   num_positive_increments = (int)((max_limit-initial_guess)/search_discretization_);
  //   num_negative_increments = (int)((initial_guess-min_limit)/search_discretization_);
  // }
  // else // no consitency limits provided
  // {
    num_positive_increments = (joint_max_vector_[free_params_[0]]-initial_guess)/search_discretization_;
    num_negative_increments = (initial_guess-joint_min_vector_[free_params_[0]])/search_discretization_;
  // }

  // -------------------------------------------------------------------------------------------------
  // Begin searching

  std::cout << "Free param is " << free_params_[0] << " initial guess is " << initial_guess << ", # positive increments: " << num_positive_increments << ", # negative increments: " << num_negative_increments << std::endl;
  // if ((search_mode & OPTIMIZE_MAX_JOINT) && (num_positive_increments + num_negative_increments) > 1000)
  //     ROS_WARN_STREAM_ONCE_NAMED("ikfast", "Large search space, consider increasing the search discretization");
  
  double best_costs = -1.0;
  std::vector<double> best_solution;
  int nattempts = 0, nvalid = 0;

  // ROS_DEBUG_STREAM_NAMED("ikfast", "Search discretization: " << search_discretization_);

  while(true)
  {
    IkSolutionList<IkReal> solutions;
    int numsol = solve(pose, vfree, solutions);

    if( numsol > 0 )
    {
      std::cout << "Found " << numsol << " solutions from IKFast" << std::endl;
      std::cout << "Value of vfree: " << vfree[0] << std::endl;
      // printf("Value of vfree: %f\n",vfree[0]);

      for(int s = 0; s < numsol; ++s)
      {
        nattempts++;
        std::vector<double> sol;
        getSolution(solutions,s,sol);

        bool obeys_limits = true;
        
        for(unsigned int i = 0; i < sol.size(); i++)
        {
          if(joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i]))
          {
            obeys_limits = false;
            break;
          }
          //ROS_INFO_STREAM_NAMED("ikfast","Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
        }

        if(obeys_limits)
        {
          getSolution(solutions,s,solution);

          ik_solutions.push_back(solution);

          // This solution is within joint limits, now check if in collision (if callback provided)
          // if(!solution_callback.empty())
          // {
          //   solution_callback(ik_pose, solution, error_code);
          // }
          // else
          // {
          //   error_code.val = error_code.SUCCESS;
          // }

          // if(error_code.val == error_code.SUCCESS)
          // {
            nvalid++;

            // if (search_mode & OPTIMIZE_MAX_JOINT)
            // {
              // Costs for solution: Largest joint motion
              double costs = 0.0;

              for(unsigned int i = 0; i < solution.size(); i++)
              {
                double d = fabs(ik_seed_state[i] - solution[i]);
                if (d > costs)
                  costs = d;
              }

              if (costs < best_costs || best_costs == -1.0)
              {
                best_costs = costs;
                best_solution = solution;
              }
          //   }
          //   else
          //     // Return first feasible solution
          //     return true;
          // }

        }
      }
    }

    if(!getCount(counter, num_positive_increments, -num_negative_increments))
    {
      // Everything searched
      // error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      break;
    }

    vfree[0] = initial_guess + search_discretization_ * counter;
    // ROS_DEBUG_STREAM_NAMED("ikfast","Attempt " << counter << " with 0th free joint having value " << vfree[0]);
  }

  std::cout << "Valid solutions: " << nvalid << "/" << nattempts << std::endl;

  if(best_costs != -1.0)
  {
    solution = best_solution;
  }
  else
  {
    solution.clear();
  }
}

bool IKFastKinematicsPlugin::getCount(int &count, const int &max_count, const int &min_count) const
{
  if(count > 0)
  {
    if(-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}



  