#include "UrdfLoader.hpp"
#include "Vector3.hpp"

void UrdfLoader::warn(Report* rep, const std::string& msg) {

	if (rep == nullptr) return;

	rep->warnings.push_back(msg);
}

std::string UrdfLoader::requiredAttr(const tinyxml2::XMLElement* el, const char* attrName, bool strict, Report* rep, const std::string& ctx)
{
    if (attrName == nullptr || attrName[0] == '\0')
        throw std::invalid_argument("[UrdfLoader] requiredAttr: attrName is null/empty. Context: " + ctx);

    if (el == nullptr) {
        const std::string msg = "[UrdfLoader] Missing element while reading attribute '" +
            std::string(attrName) + "'. Context: " + ctx;

        if (strict) {
            throw std::runtime_error(msg);
        }

        warn(rep, msg);
        return "";
    }

    const char* value = el->Attribute(attrName);
    if (value == nullptr || value[0] == '\0') {
        const std::string msg = "[UrdfLoader] Missing/empty attribute '" +
            std::string(attrName) + "'. Context: " + ctx;

        if (strict) {
            throw std::runtime_error(msg);
        }

        warn(rep, msg);
        return "";
    }

    return std::string(value);
}

const tinyxml2::XMLElement* UrdfLoader::requiredChild(const tinyxml2::XMLElement* parent, const char* name, bool strict, Report* rep, const std::string& ctx) {
    if (name == nullptr || name[0] == '\0')
        throw std::invalid_argument("[UrdfLoader] requiredChild: name argument is null/empty. Context: " + ctx);

    if (parent == nullptr) {
        const std::string msg = "[UrdfLoader] requiredChild: parent element is null; cannot read child <" +
            std::string(name) + ">. Context: " + ctx;

        if (strict) {
            throw std::runtime_error(msg);
        }

        warn(rep, msg);
        return nullptr;
    }

    const tinyxml2::XMLElement* child = parent->FirstChildElement(name);

    if (child == nullptr) {

        const std::string msg = "[UrdfLoader] Missing required child element <" + std::string(name) +
            ">. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return nullptr;
    }

    return child;
}

const tinyxml2::XMLElement* UrdfLoader::optionalChild(const tinyxml2::XMLElement* parent, const char* name) {
    
    if (parent == nullptr) return nullptr;

    return parent->FirstChildElement(name);
}

Vector3 UrdfLoader::parseVec3Text(const char* text, const Vector3& def, bool strict, Report* rep, const std::string& ctx) {
    
    auto handleError = [&](const std::string& msg) -> Vector3 {
        if (strict) {
            throw std::runtime_error(msg);
        }
        UrdfLoader::warn(rep, msg);
        return def;
        };

    if (text == nullptr) {
        return handleError("[UrdfLoader] Missing vec3 text (nullptr). Context: " + ctx);
    }

    std::istringstream iss(text);

    iss >> std::ws;
    if (iss.eof()) {
        return handleError("[UrdfLoader] Empty vec3 text. Context: " + ctx);
    }

    double x = 0.0, y = 0.0, z = 0.0;
    if (!(iss >> x >> y >> z)) {
        return handleError("[UrdfLoader] Cannot parse vec3 from '" + std::string(text) +
            "'. Expected three numbers. Context: " + ctx);
    }

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return handleError("[UrdfLoader] Non-finite vec3 values in '" + std::string(text) +
            "'. Context: " + ctx);
    }

    iss >> std::ws;
    if (!iss.eof()) {
        const std::string msg =
            "[UrdfLoader] Extra tokens in vec3 text '" + std::string(text) +
            "'. Using first three values. Context: " + ctx;

        if (strict) {
            throw std::runtime_error(msg);
        }
        UrdfLoader::warn(rep, msg);
    }

    return Vector3(x, y, z);
}

KinematicModel::JointType UrdfLoader::parseJointType(const std::string& typeStr, bool treatContinuousAsRevolute) {
    if (typeStr.empty())
        throw std::invalid_argument("[UrdfLoader] parseJointType: empty joint type string.");

    if (typeStr == "revolute")
        return KinematicModel::JointType::Revolute;

    else if (typeStr == "fixed")
        return KinematicModel::JointType::Fixed;

    else if (typeStr == "prismatic")
        return KinematicModel::JointType::Prismatic;

    else if (typeStr == "continuous") {
        if (treatContinuousAsRevolute) {
            return KinematicModel::JointType::Revolute;
        }
        throw std::runtime_error("[UrdfLoader] Joint type 'continuous' not supported by current configuration.");
    }

    throw std::runtime_error("[UrdfLoader] Unsupported joint type: " + typeStr);
}

Matrix4 UrdfLoader::parseOrigin(const tinyxml2::XMLElement* jointEl, bool strict, Report* rep, const std::string& ctx = "") {

    if (jointEl == nullptr) {
        const std::string msg = "[UrdfLoader] parseOrigin: joint element is null. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return Matrix4::identity();
    }

    const tinyxml2::XMLElement* originEl = optionalChild(jointEl, "origin");

    if (!originEl)
        return Matrix4::identity();

    const Vector3 defZero(0.0, 0.0, 0.0);

    Vector3 t = defZero;
    Vector3 rpy = defZero;

    const char* xyzText = originEl->Attribute("xyz");

    if (xyzText != nullptr)
        t = parseVec3Text(xyzText, defZero, strict, rep, ctx + " / origin/xyz");


    const char* rpyText = originEl->Attribute("rpy");

    if (rpyText == nullptr)
        rpy = parseVec3Text(rpyText, defZero, strict, rep, ctx + " / origin/rpy");

    const Matrix3 R = Matrix3::fromEuler(rpy.getX(), rpy.getY(), rpy.getZ());
    return Matrix4(R, t);
}

Vector3 UrdfLoader::parseAxis(const tinyxml2::XMLElement* jointEl, bool normalizeAxis, bool strict, Report* rep, const std::string& ctx = "") {
    const Vector3 defAxis(1.0, 0.0, 0.0);

    if (!jointEl) {
        const std::string msg =
            "[UrdfLoader] parseAxis: joint element is null. Context: " + ctx;

        if (strict) {
            throw std::runtime_error(msg);
        }

        warn(rep, msg);
        return defAxis;
    }

    const tinyxml2::XMLElement* axisEl = optionalChild(jointEl, "axis");
    if (!axisEl) {
        return defAxis;
    }

    const char* xyzText = axisEl->Attribute("xyz");
    Vector3 axis = defAxis;

    if (xyzText != nullptr) {
        axis = parseVec3Text(xyzText, defAxis, strict, rep, ctx + " / axis/xyz");
    }

    if (normalizeAxis) {
        const double len = axis.length();
        if (len < 1e-12) {
            const std::string msg =
                "[UrdfLoader] parseAxis: axis has near-zero length. Context: " + ctx;

            if (strict) {
                throw std::runtime_error(msg);
            }

            warn(rep, msg);
            return defAxis;
        }

        axis.normalize();
    }

    return axis;
}

void UrdfLoader::parseLimit(const tinyxml2::XMLElement* jointEl, KinematicModel::JointType jointType, KinematicModel::JointLimit& outLimit, bool strict, Report* rep, const std::string& ctx = "") {
    outLimit.hasLimits = false;
    outLimit.lower = 0.0;
    outLimit.upper = 0.0;

    if (jointType == KinematicModel::JointType::Fixed) return;

    if (!jointEl) {
        const std::string msg =
            "[UrdfLoader] parseLimit: joint element is null. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return;
    }

    const tinyxml2::XMLElement* limitEl = optionalChild(jointEl, "limit");
    if (!limitEl) {
        const std::string msg =
            "[UrdfLoader] Missing <limit> for non-fixed joint. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return;
    }

    const char* lowerStr = limitEl->Attribute("lower");
    const char* upperStr = limitEl->Attribute("upper");

    if (!lowerStr || !upperStr || lowerStr[0] == '\0' || upperStr[0] == '\0') {
        const std::string msg =
            "[UrdfLoader] <limit> missing/empty 'lower' or 'upper'. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return;
    }

    double lower = 0.0, upper = 0.0;
    try {
        lower = std::stod(lowerStr);
        upper = std::stod(upperStr);
    }
    catch (const std::exception&) {
        const std::string msg =
            "[UrdfLoader] Cannot parse <limit lower/upper> as double. lower='" +
            std::string(lowerStr) + "', upper='" + std::string(upperStr) +
            "'. Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return;
    }

    if (lower > upper) {
        const std::string msg =
            "[UrdfLoader] Invalid <limit>: lower > upper (" + std::to_string(lower) +
            " > " + std::to_string(upper) + "). Context: " + ctx;

        if (strict)
            throw std::runtime_error(msg);

        warn(rep, msg);
        return;
    }

    outLimit.hasLimits = true;
    outLimit.lower = lower;
    outLimit.upper = upper;
}