#pragma once

#include <memory>
#include <vector>
#include <cstddef>
#include <string>
#include <algorithm>
#include <utility>

#include "KinematicModel.hpp"
#include "Matrix4.hpp"
#include "Matrix.hpp"

class RobotRunTime {

public:

	using LinkId = KinematicModel::LinkId;
	using JointId = KinematicModel::JointId;

	explicit RobotRunTime(std::shared_ptr<const KinematicModel> model);

	size_t dof() const;

	const std::vector<double>& q()  const;
	const std::vector<double>& qd() const;

	void setQ(const std::vector<double>& q);
	void setQd(const std::vector<double>& qd);

	void setJointPosition(const std::string& jointName, double value);
	void setJointPosition(JointId id, double value);

	void stepKinematic(double dt, const std::vector<double>& qd_cmd);

	const Matrix4& linkPoseWorld(LinkId link) const;

	Matrix4 linkPoseWorld(const std::string& linkName) const;

private:

	void updateKinematicsIfNeeded_() const;

	void clampToLimits_();

	void markDirty_();

private:

	std::shared_ptr<const KinematicModel> model_;

	std::vector<JointId> activeJoints_;
	std::vector<int> qIndexOfJoint_;

	std::vector<double> q_;
	std::vector<double> qd_;

	mutable bool kinematicsDirty_ = true;
	mutable std::vector<Matrix4> T_world_link_;
};