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