#pragma once

#include <string>
#include <vector>
#include <optional>

#include "KinematicModel.hpp"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <cmath>

namespace tinyxml2 {
	class XMLDocument;
	class XMLElement;
}

class UrdfLoader final {
public:
	struct Options {
		bool autoCreateMissingLinks = true;
		bool keepFixedJoints = true;
		bool treatContinuousAsRevolute = true;
		bool normalizeAxis = true;
		std::optional<std::string> rootLinkOverride = std::nullopt;
		bool strict = false;
	};

	struct Report {
		std::string robotName;
		std::string rootLinkName;

		std::size_t links = 0;
		std::size_t joints = 0;
		std::size_t activeJoints = 0;

		std::vector<std::string> warnings;
	};

public:
	UrdfLoader() = default;
	explicit UrdfLoader(const Options& opt) : opt_(opt) {}

	void setOptions(const Options& opt) { opt_ = opt; }
	const Options& options() const { return opt_; }

	KinematicModel loadFromFile(const std::string& urdfPath, Report* outReport = nullptr) const;
	KinematicModel loadFromString(const std::string& urdfXml, Report* outReport = nullptr) const;

private:
	Options opt_;

private:
	static const tinyxml2::XMLElement* findRobotElement(const tinyxml2::XMLDocument& doc);

	static std::string readRobotName(const tinyxml2::XMLElement* robotEl);

	void parseLinks(const tinyxml2::XMLElement* robotEl, KinematicModel& model, Report* rep) const;
	void parseJoints(const tinyxml2::XMLElement* robotEl, KinematicModel& model, Report* rep) const;

	void chooseAndSetRoot(KinematicModel& model, Report* rep) const;

	void fillStats(const KinematicModel& model, Report* rep) const;

	static const tinyxml2::XMLElement* optionalChild(const tinyxml2::XMLElement* parent, const char* name);
	
	static const tinyxml2::XMLElement* requiredChild(
		const tinyxml2::XMLElement* parent,
		const char* name,
		bool strict,
		Report* rep,
		const std::string& ctx
	);

	static std::string requiredAttr(
		const tinyxml2::XMLElement* el,
		const char* attrName,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static void warn(Report* rep, const std::string& msg);

	static KinematicModel::JointType parseJointType(
		const std::string& typeStr,
		bool treatContinuousAsRevolute
	);

	static Vector3 parseVec3Text(
		const char* text,
		const Vector3& def,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static Matrix4 parseOrigin(
		const tinyxml2::XMLElement* jointEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static Vector3 parseAxis(
		const tinyxml2::XMLElement* jointEl,
		bool normalizeAxis,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static void parseLimit(
		const tinyxml2::XMLElement* jointEl,
		KinematicModel::JointType jointType,
		KinematicModel::JointLimit& outLimit,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static double parseDoubleAttr(
		const tinyxml2::XMLElement* el,
		const char* attrName,
		double def,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static Vector3 parseTripleAttr(
		const tinyxml2::XMLElement* El, 
		const char* attrName, bool strict, 
		Report* rep, 
		const std::string& ctx
	);

	static std::optional<KinematicModel::Inertial> parseInertial(
		const tinyxml2::XMLElement* linkEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static std::optional<KinematicModel::Geometry> parseGeometry(
		const tinyxml2::XMLElement* geometryEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static std::optional<KinematicModel::Collision> parseCollision(
		const tinyxml2::XMLElement* collisionEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static std::optional<KinematicModel::Material> parseMaterial(
		const tinyxml2::XMLElement* materialEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);

	static std::optional<KinematicModel::Visual> parseVisual(
		const tinyxml2::XMLElement* visualEl,
		bool strict,
		Report* rep,
		const std::string& ctx = ""
	);
};
