#include "KinematicModel.hpp"
#include <stdexcept>

using LinkId = std::size_t;
using JointId = std::size_t;

LinkId KinematicModel::addLink(const std::string& name) {

	if (name.empty())
		throw std::invalid_argument("[KinematicModel] Link name cannot be empty.");

	if (linkNameToId_.find(name) != linkNameToId_.end())
		throw std::invalid_argument("[KinematicModel] Link with this name already exists: " + name);
	
	const LinkId id = links_.size();
	links_.push_back(Link{ name });

	linkNameToId_[name] = id;

	childJointsByLink_.emplace_back();
	parentJointOfLink_.push_back(std::nullopt);

	if (!rootLink_.has_value()) {
		rootLink_ = id;
	}

	return id;
}

KinematicModel::JointId KinematicModel::addJoint(const Joint& jointIn)
{
    if (jointIn.name.empty()) {
        throw std::invalid_argument("[KinematicModel] Joint name cannot be empty.");
    }
    if (jointNameToId_.find(jointIn.name) != jointNameToId_.end()) {
        throw std::runtime_error("[KinematicModel] Duplicate joint name: " + jointIn.name);
    }

    if (jointIn.parentLink >= links_.size()) {
        throw std::out_of_range("[KinematicModel] Joint '" + jointIn.name +
            "': parentLink id out of range.");
    }
    if (jointIn.childLink >= links_.size()) {
        throw std::out_of_range("[KinematicModel] Joint '" + jointIn.name +
            "': childLink id out of range.");
    }
    if (jointIn.parentLink == jointIn.childLink) {
        throw std::runtime_error("[KinematicModel] Joint '" + jointIn.name +
            "': parentLink and childLink must be different.");
    }

    if (jointIn.childLink < parentJointOfLink_.size() &&
        parentJointOfLink_[jointIn.childLink].has_value()) {
        throw std::runtime_error("[KinematicModel] Joint '" + jointIn.name +
            "': child link already has a parent joint.");
    }

    Joint joint = jointIn;

    constexpr double kEps = 1e-12;

    if (joint.type == JointType::Fixed) {
        joint.axis = Vector3(1.0, 0.0, 0.0);
        joint.limit.hasLimits = false;
    }
    else if (joint.type == JointType::Revolute || joint.type == JointType::Prismatic) {
        const double ax = joint.axis.getX();
        const double ay = joint.axis.getY();
        const double az = joint.axis.getZ();
        const double len = std::sqrt(ax * ax + ay * ay + az * az);

        if (len < kEps) {
            throw std::runtime_error("[KinematicModel] Joint '" + joint.name +
                "': axis length is zero (invalid for revolute/prismatic).");
        }

        joint.axis = Vector3(ax / len, ay / len, az / len);

        if (joint.limit.hasLimits) {
            if (!std::isfinite(joint.limit.lower) || !std::isfinite(joint.limit.upper)) {
                throw std::runtime_error("[KinematicModel] Joint '" + joint.name +
                    "': limits contain NaN/Inf.");
            }
            if (joint.limit.lower > joint.limit.upper) {
                throw std::runtime_error("[KinematicModel] Joint '" + joint.name +
                    "': invalid limits (lower > upper).");
            }
        }
    }
    else {
        throw std::runtime_error("[KinematicModel] Joint '" + joint.name +
            "': unsupported joint type.");
    }

    if (joint.parentLink >= childJointsByLink_.size()) {
        throw std::runtime_error("[KinematicModel] Internal error: childJointsByLink_ not initialized for links.");
    }

    if (joint.childLink >= parentJointOfLink_.size()) {
        throw std::runtime_error("[KinematicModel] Internal error: parentJointOfLink_ not initialized for links.");
    }

    const JointId id = joints_.size();
    joints_.push_back(joint);
    jointNameToId_[joint.name] = id;

    childJointsByLink_[joint.parentLink].push_back(id);

    parentJointOfLink_[joint.childLink] = id;

    return id;
}

void KinematicModel::setRoot(LinkId root) {

    if (links_.empty())
        throw std::logic_error("[KinematicModel] Cannot set root: no links in model.");

    if (root >= links_.size())
        throw std::out_of_range("[KinematicModel] Invalid root id (out of range).");

    if (parentJointOfLink_.size() != links_.size())
        throw std::logic_error("[KinematicModel] Internal error: parentJointOfLink_ not initialized for all links.");

    if (parentJointOfLink_[root].has_value())
        throw std::invalid_argument("[KinematicModel] Root can not have a parent joint.");

    if (rootLink_.has_value() && rootLink_.value() == root) {
        std::cerr << "[KinematicModel] root with this id has been already set.";
        return;
    }

    if (rootLink_.has_value()) {
        std::cerr << "[KinematicModel] Warning: changing root from '"
            << links_[rootLink_.value()].name << "' to '" << links_[root].name << "'.\n";
    }
    else std::cerr << "[KinematicModel] Adding new root to the kinematic model.";

    rootLink_ = root;
}

bool KinematicModel::hasRoot() const {

    return rootLink_.has_value();
}

LinkId KinematicModel::root() const {

    if (!rootLink_.has_value()) {
        throw std::logic_error("[KinematicModel] Root hasn't been set yet.");
    }

    return rootLink_.value();
}

bool KinematicModel::hasLink(const std::string& name) const {
    return (linkNameToId_.find(name) != linkNameToId_.end());
}

bool KinematicModel::hasJoint(const std::string& name) const {
    return (jointNameToId_.find(name) != jointNameToId_.end());
}

LinkId KinematicModel::linkId(const std::string& name) const {

    if (name.empty())
        throw std::invalid_argument("[KinematicModel] Link name cannot be empty.");

    if (!hasLink(name))
        throw std::invalid_argument("[KinematicModel] Link with given name doesn't exist.");

    auto it = linkNameToId_.find(name);

    return it->second;
}

JointId KinematicModel::jointId(const std::string& name) const {

    if (name.empty())
        throw std::invalid_argument("[KinematicModel] Joint name cannot be empty.");

    if (!hasJoint(name))
        throw std::invalid_argument("[KinematicModel] Joint with given name doesn't exist.");

    auto it = jointNameToId_.find(name);

    return it->second;
}

const KinematicModel::Link& KinematicModel::link(LinkId id) const {
    if (id >= links_.size())
        throw std::out_of_range("[KinematicModel] Given link ID is invalid (out of range).");

    return links_[id];
}

const KinematicModel::Joint& KinematicModel::joint(JointId id) const {
    if (id >= joints_.size())
        throw std::out_of_range("[KinematicModel] Given joint id is invalid (out of range).");

    return joints_[id];
}

std::size_t KinematicModel::linkCount() const {
    return links_.size();
}

std::size_t KinematicModel::jointCount() const {
    return joints_.size();
}

std::size_t KinematicModel::activeJointCount() const {

    size_t res = 0;

    if (joints_.empty()) return res;

    for (const auto& joint : joints_) {
        if (joint.type != JointType::Fixed) ++res;
    }

    return res;
}