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

const std::vector<JointId>& KinematicModel::childJoints(LinkId parent) const {

    if (childJointsByLink_.size() != links_.size())
        throw std::logic_error("[KinematicModel] Internal error: not initialized for all links.");

    if (parent >= links_.size())
        throw std::out_of_range("[KinematicModel] Given parent ID is invalid.");

    return childJointsByLink_[parent];
}

std::optional<KinematicModel::JointId> KinematicModel::parentJoint(LinkId child) const {
    if (child >= links_.size()) {
        throw std::out_of_range("[KinematicModel] Given child ID is invalid (out of range).");
    }

    if (parentJointOfLink_.size() != links_.size()) {
        throw std::logic_error("[KinematicModel] Internal error: parentJointOfLink_ not initialized for all links.");
    }

    return parentJointOfLink_[child];
}

std::optional<KinematicModel::LinkId> KinematicModel::parentLink(LinkId child) const {
    if (child >= links_.size()) {
        throw std::out_of_range("[KinematicModel] Given child ID is invalid (out of range).");
    }

    if (parentJointOfLink_.size() != links_.size()) {
        throw std::logic_error("[KinematicModel] Internal error: parentJointOfLink_ not initialized for all links.");
    }

    if (!parentJointOfLink_[child].has_value()) {
        return std::nullopt;
    }

    const JointId pj = parentJointOfLink_[child].value();

    if (pj >= joints_.size()) {
        throw std::logic_error("[KinematicModel] Internal error: parent joint id is out of range.");
    }

    return joints_[pj].parentLink;
}

std::vector<JointId> KinematicModel::chain(LinkId from, LinkId to) const {

    if (from >= links_.size())
        throw std::out_of_range("[KinematicModel] Given from value is invalid (out of range).");

    if (to >= links_.size())
        throw std::out_of_range("[KinematicModel] Given to value is invalid (out of range).");

    if (parentJointOfLink_.size() != links_.size())
        throw std::logic_error("[KinematicModel] Internal error: parentJointOfLink_ not initialized for all links.");

    if (to == from)
        return { };

    std::vector<JointId> jointsPath;
    LinkId current = to;

    while (current != from) {
        const auto pjOpt = parentJointOfLink_[current];
        if (!pjOpt.has_value()) {
            throw std::runtime_error("[KinematicModel] chain(): no path from 'from' to 'to' (from is not an ancestor of to).");
        }

        const JointId pj = pjOpt.value();

        if (pj >= joints_.size()) {
            throw std::logic_error("[KinematicModel] chain(): internal error: parent joint id out of range.");
        }
        if (joints_[pj].childLink != current) {
            throw std::logic_error("[KinematicModel] chain(): internal error: parentJointOfLink_ inconsistent with joints_ (childLink mismatch).");
        }

        jointsPath.push_back(pj);
        current = joints_[pj].parentLink;
    }
    std::reverse(jointsPath.begin(), jointsPath.end());
    return jointsPath;
}

std::vector<JointId> KinematicModel::activeJoints() const {
    std::vector<JointId> res;

    res.reserve(joints_.size());

    for (size_t i = 0; i < joints_.size(); i++) {
        if (joints_[i].type != JointType::Fixed) {
            res.push_back(i);
        }
    }

    return res;
}

void KinematicModel::validate() const {

    if (links_.empty()) {
        throw std::logic_error("[KinematicModel] validate(): no links in model.");
    }

    if (childJointsByLink_.size() != links_.size()) {
        throw std::logic_error("[KinematicModel] validate(): childJointsByLink_ not initialized for all links.");
    }

    if (parentJointOfLink_.size() != links_.size()) {
        throw std::logic_error("[KinematicModel] validate(): parentJointOfLink_ not initialized for all links.");
    }

    if (!rootLink_.has_value()) {
        throw std::logic_error("[KinematicModel] validate(): root is not set.");
    }

    const LinkId root = rootLink_.value();
    if (root >= links_.size()) {
        throw std::logic_error("[KinematicModel] validate(): root id out of range.");
    }

    if (parentJointOfLink_[root].has_value()) {
        throw std::logic_error("[KinematicModel] validate(): root link has a parent joint (invalid tree).");
    }

    for (LinkId i = 0; i < links_.size(); ++i) {
        const auto it = linkNameToId_.find(links_[i].name);
        if (it == linkNameToId_.end() || it->second != i) {
            throw std::logic_error("[KinematicModel] validate(): linkNameToId_ inconsistent for link: " + links_[i].name);
        }
    }

    for (JointId j = 0; j < joints_.size(); ++j) {
        const auto it = jointNameToId_.find(joints_[j].name);
        if (it == jointNameToId_.end() || it->second != j) {
            throw std::logic_error("[KinematicModel] validate(): jointNameToId_ inconsistent for joint: " + joints_[j].name);
        }
    }

    constexpr double kEps = 1e-12;

    for (JointId j = 0; j < joints_.size(); ++j) {
        const Joint& J = joints_[j];

        if (J.parentLink >= links_.size()) {
            throw std::logic_error("[KinematicModel] validate(): joint '" + J.name + "' parentLink out of range.");
        }
        if (J.childLink >= links_.size()) {
            throw std::logic_error("[KinematicModel] validate(): joint '" + J.name + "' childLink out of range.");
        }
        if (J.parentLink == J.childLink) {
            throw std::logic_error("[KinematicModel] validate(): joint '" + J.name + "' parentLink == childLink.");
        }

        if (!parentJointOfLink_[J.childLink].has_value() || parentJointOfLink_[J.childLink].value() != j) {
            throw std::logic_error("[KinematicModel] validate(): joint '" + J.name +
                "' inconsistent: parentJointOfLink_[child] does not match.");
        }

        if (J.type == JointType::Revolute || J.type == JointType::Prismatic) {
            const double ax = J.axis.getX();
            const double ay = J.axis.getY();
            const double az = J.axis.getZ();
            const double len = std::sqrt(ax * ax + ay * ay + az * az);

            if (len < kEps) {
                throw std::logic_error("[KinematicModel] validate(): joint '" + J.name + "' has zero axis.");
            }

            if (J.limit.hasLimits && J.limit.lower > J.limit.upper) {
                throw std::logic_error("[KinematicModel] validate(): joint '" + J.name + "' has invalid limits (lower > upper).");
            }
        }
    }

    std::vector<unsigned char> color(links_.size(), 0);

    std::vector<std::pair<LinkId, std::size_t>> stack;
    stack.emplace_back(root, 0);
    color[root] = 1;

    while (!stack.empty()) {
        auto& top = stack.back();
        LinkId u = top.first;
        std::size_t& idx = top.second;

        const auto& childrenJ = childJointsByLink_[u];

        if (idx >= childrenJ.size()) {
            color[u] = 2;
            stack.pop_back();
            continue;
        }

        const JointId jid = childrenJ[idx++];
        if (jid >= joints_.size()) {
            throw std::logic_error("[KinematicModel] validate(): child joint id out of range in childJointsByLink_.");
        }

        const LinkId v = joints_[jid].childLink;

        if (color[v] == 1) {
            throw std::logic_error("[KinematicModel] validate(): cycle detected in kinematic tree.");
        }
        if (color[v] == 0) {
            color[v] = 1;
            stack.emplace_back(v, 0);
        }
    }

    for (LinkId i = 0; i < links_.size(); ++i) {
        if (color[i] == 0) {
            throw std::logic_error("[KinematicModel] validate(): link not reachable from root: " + links_[i].name);
        }
    }
}

