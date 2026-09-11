// Planar forward kinematics using one joint angle per segment.
Eigen::Matrix4d SurgicalTool::computeTransformationMatrix(
  const double& theta)
{
  double R = this->surgicaltool_.arc * 3;
  double L = this->surgicaltool_.arc_center_to_seg_center;
  double l_theta = R / std::cos(theta / 2.0) - L;
  double p_x = l_theta * (1.0 + std::cos(theta));
  double p_y = l_theta * std::sin(theta);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(theta);
  T(0, 1) = -std::sin(theta);
  T(1, 0) = std::sin(theta);
  T(1, 1) = std::cos(theta);
  T(0, 3) = p_x;
  T(1, 3) = p_y;

  return T;
}

std::vector<Eigen::Matrix4d>
SurgicalTool::computeBaseToJointsTransformationMatrices(
  const std::vector<double>& joint_angles)
{
  std::vector<Eigen::Matrix4d> transform_matrices;
  Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();

  for (int i = 0; i < this->surgicaltool_.num_joint; ++i) {
    Eigen::Matrix4d T_next = computeTransformationMatrix(joint_angles[i]);
    T_current = T_current * T_next;
    transform_matrices.push_back(T_current);
  }

  return transform_matrices;
}

