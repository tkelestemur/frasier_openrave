# frasier_openrave

### TODO:
- [x] cannot set goal angle for base_t_joint
- [x] fix viewer (change to osg viewer)
- [ ] redundant ik solver from openrave
- [ ] point cloud updater using ROS
- [x] IK using trajopt
- [x] planning using trajopt


### Installation
TrajOpt: `cmake .. -DGUROBI_LIBRARY=/opt/gurobi752/linux64/lib/libgurobi75.so`


trajopt::TrajOptProbPtr prob(new trajopt::TrajOptProb(n_steps, pci.rad));

int n_dof = pci.rad->GetDOF();
trajopt::DblVec cur_dofvals = pci.rad->GetDOFValues();

if (start_fixed) {
  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(prob->m_traj_vars(0,j)), cur_dofvals[j]), EQ);
  }
}
Vector4d wxyz = {1, 0, 0, 0};
Vector3d xyz = {1, 0.8, .9};
std:string eef_name = "hand_palm_link";
VectorOfVectorPtr f(new CartPoseErrCalculator(toRaveTransform(wxyz, xyz), prob->GetRAD(), link));



// pci.basic_info.start_fixed = true;
// pci.basic_info.n_steps = 10;
// pci.basic_info.manip = "whole_body";
//
// pci.init_info.type = trajopt::InitInfo::STATIONARY;
//
// trajopt::JointVelCostInfo joint_vel_cost;

// trajopt::TermInfoPtr pose_cost;
// pose_cost->name = "eef_pose";
// pose_cost->type = "pose";
// pose_cost->timestep = 9;
// pose_cost->xyz = {2.25, 0.0, 1.1};

// std::cout << "constraint info: " << pci.cnt_infos[0]->xyz << std::endl;
