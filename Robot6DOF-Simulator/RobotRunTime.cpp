#include "RobotRunTime.hpp"

RobotRunTime::RobotRunTime(std::shared_ptr<const KinematicModel> model) : model_(std::move(model)) {

	if (!model_)
		throw std::runtime_error("[RobotRunTime] null model.");

	model_->validate();

	activeJoints_ = model_->activeJoints();
	
	q_.assign(activeJoints_.size(), 0.0);
	qd_.assign(activeJoints_.size(), 0.0);
	qIndexOfJoint_.assign(model_->jointCount(), -1);

	for (std::size_t i = 0; i < activeJoints_.size(); ++i) {
		const JointId jid = activeJoints_[i];

		if (jid >= qIndexOfJoint_.size()) {
			throw std::logic_error("[RobotRuntime] active joint id out of range (model not valid).");
		}

		qIndexOfJoint_[jid] = static_cast<int>(i);
	}

	for (std::size_t i = 0; i < activeJoints_.size(); ++i) {
		const auto& J = model_->joint(activeJoints_[i]);
		if (J.limit.hasLimits) {
			q_[i] = std::clamp(q_[i], J.limit.lower, J.limit.upper);
		}
	}

	T_world_link_.assign(model_->linkCount(), Matrix4::identity());

	kinematicsDirty_ = true;
}

void RobotRunTime::setQ(const std::vector<double>& q) {

	if (q.size() != activeJoints_.size())
		throw std::invalid_argument("[RobotRuntime] setQ(): wrong size.");

	for (double v : q) {
		if (!std::isfinite(v)) {
			throw std::invalid_argument("[RobotRuntime] setQ(): q contains NaN/Inf.");
		}
	}

	q_ = q;

	for (std::size_t i = 0; i < activeJoints_.size(); ++i) {
		const JointId jid = activeJoints_[i];
		const auto& J = model_->joint(jid);

		if (J.limit.hasLimits) {
			q_[i] = std::clamp(q_[i], J.limit.lower, J.limit.upper);
		}
	}

	kinematicsDirty_ = true;
}