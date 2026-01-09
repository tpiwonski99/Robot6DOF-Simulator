#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <optional>
#include <stdexcept>

#include "Vector3.hpp"
#include "Matrix4.hpp"
#include "Matrix.hpp"
#include "Vector3.hpp"

class KinematicModel {

public:

	using LinkId = std::size_t;
	using JointId = std::size_t;

	enum class JointType {
		Fixed,
		Revolute,
		Prismatic
	};

	struct Link {
		std::string name;
		// TO DO: VISUALS, COLLISIONS, INTERTIAL
	};

	struct JointLimit {
		bool hasLimits = false;
		double lower = 0.0;
		double upper = 0.0;
	};

	struct WorldKinematics {
		std::vector<Matrix4> T_world_link;   
		std::vector<Matrix4> T_world_joint;  
	};

	struct Joint {
		std::string name;

		JointType type = JointType::Fixed;

		LinkId parentLink = 0;
		LinkId childLink = 0;

		Matrix4 origin;

		Vector3 axis;

		JointLimit limit;
	};

private:
	
	std::vector<Link> links_;
	std::vector<Joint> joints_;

	std::unordered_map<std::string, LinkId> linkNameToId_;
	std::unordered_map<std::string, JointId> jointNameToId_;

	std::vector<std::vector<JointId>> childJointsByLink_;
	std::vector<std::optional<JointId>> parentJointOfLink_;

	std::optional<LinkId> rootLink_;

public: 

	LinkId  addLink(const std::string& name);
	JointId addJoint(const Joint& joint);

	void setRoot(LinkId root);
	bool hasRoot() const;
	LinkId root() const;

	bool hasLink(const std::string& name) const;
	bool hasJoint(const std::string& name) const;

	LinkId  linkId(const std::string& name) const;
	JointId jointId(const std::string& name) const;

	const Link& link(LinkId id) const;
	const Joint& joint(JointId id) const;

	std::size_t linkCount() const;
	std::size_t jointCount() const;

	std::size_t activeJointCount() const;

	const std::vector<JointId>& childJoints(LinkId parent) const;

	std::optional<JointId> parentJoint(LinkId child) const;
	std::optional<LinkId>  parentLink(LinkId child) const;

	std::vector<JointId> chain(LinkId from, LinkId to) const;

	std::vector<JointId> activeJoints() const;

	void validate() const;

	// qActive[i] = activeJoints()[i]
	std::vector<Matrix4> forwardKinematicsAll(const std::vector<double>& qActive) const;
	Matrix4 poseWorld(LinkId link, const std::vector<double>& qActive) const;

	WorldKinematics worldKinematics(const std::vector<double>& qActive) const;

	Matrix jacobian(LinkId endEff, const std::vector<double>& qActive) const;
};
