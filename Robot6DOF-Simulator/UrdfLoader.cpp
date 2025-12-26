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

        if (strict)
            throw std::runtime_error(msg);
     

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

const tinyxml2::XMLElement* UrdfLoader::findRobotElement(const tinyxml2::XMLDocument& doc) {

    const tinyxml2::XMLElement* robotEl = doc.FirstChildElement("robot");
    
    if (!robotEl)
        throw std::runtime_error("[UrdfLoader] Missing <robot> root element.");

    return robotEl;
}

std::string UrdfLoader::readRobotName(const tinyxml2::XMLElement* robotEl) {

    if (!robotEl)
        throw std::invalid_argument("");

    const char* name = robotEl->Attribute("name");

    if (!name || name[0] == '\0')
        return "";
    
    return std::string(name);
}

void UrdfLoader::parseLinks(const tinyxml2::XMLElement* robotEl, KinematicModel& model, Report* rep) const {

    if (!robotEl) {
        const std::string msg = "[UrdfLoader] parseLinks: robot element is null.";
        
        if (opt_.strict)
            throw std::runtime_error(msg)
                ;
        warn(rep, msg);
        return;
    }

    for (const tinyxml2::XMLElement* linkEl = robotEl->FirstChildElement("link"); linkEl != nullptr; linkEl = linkEl->NextSiblingElement("link")) {
        
        const std::string ctx = "link";

        const std::string name = requiredAttr(linkEl, "name", opt_.strict, rep, ctx);

        if (name.empty())
            continue;

        try {
            model.addLink(name);
        }

        catch (const std::exception& e) {
            const std::string msg =
                "[UrdfLoader] parseLinks: cannot add link '" + name + "': " + e.what();

            if (opt_.strict) {
                throw;
            }

            warn(rep, msg);
            continue;
        }
    }
}

void UrdfLoader::parseJoints(const tinyxml2::XMLElement* robotEl, KinematicModel& model, Report* rep) const
{

    /*
        struct Joint {
            std::string name;

            JointType type = JointType::Fixed;

            LinkId parentLink = 0;
            LinkId childLink = 0;

            Matrix4 origin;

            Vector3 axis;

            JointLimit limit;
        };
    */  

    if (!robotEl) {
        const std::string msg = "[UrdfLoader] parseJoints: robot element is null.";
        
        if (opt_.strict) throw std::runtime_error(msg);
        
        warn(rep, msg);
        return;
    }

    for (const tinyxml2::XMLElement* jointEl = robotEl->FirstChildElement("joint");
        jointEl != nullptr;
        jointEl = jointEl->NextSiblingElement("joint"))
    {
        const char* jointNameC = jointEl->Attribute("name");
        const std::string jointName = (jointNameC ? std::string(jointNameC) : "");
        const std::string ctx = jointName.empty() ? "joint" : ("joint: " + jointName);

        const std::string name = requiredAttr(jointEl, "name", opt_.strict, rep, ctx);
        
        if (name.empty())
            continue;

        const std::string typeStr = requiredAttr(jointEl, "type", opt_.strict, rep, ctx);
        
        if (typeStr.empty())
            continue;
        
        const bool isContinuous = (typeStr == "continuous");

        KinematicModel::JointType type;
        
        try {
            type = parseJointType(typeStr, opt_.treatContinuousAsRevolute);
        }
        
        catch (const std::exception& e) {
            const std::string msg = "[UrdfLoader] " + ctx + ": " + e.what();
            
            if (opt_.strict) throw;
            
            warn(rep, msg);
            continue;
        }

        if (!opt_.keepFixedJoints && type == KinematicModel::JointType::Fixed) {
            continue;
        }

        const tinyxml2::XMLElement* parentEl = requiredChild(jointEl, "parent", opt_.strict, rep, ctx + " / parent");
        const tinyxml2::XMLElement* childEl = requiredChild(jointEl, "child", opt_.strict, rep, ctx + " / child");

        const std::string parentName = requiredAttr(parentEl, "link", opt_.strict, rep, ctx + " / parent");
        const std::string childName = requiredAttr(childEl, "link", opt_.strict, rep, ctx + " / child");

        if (parentName.empty() || childName.empty())
            continue;

        if (!model.hasLink(parentName)) {
            if (opt_.autoCreateMissingLinks) {
                
                try {
                    model.addLink(parentName);
                    warn(rep, "[UrdfLoader] " + ctx + ": auto-created missing parent link '" + parentName + "'.");
                }

                catch (const std::exception& e) {
                    const std::string msg =
                        "[UrdfLoader] " + ctx + ": cannot auto-create parent link '" + parentName + "': " + e.what();
                    
                    if (opt_.strict) throw;
                    
                    warn(rep, msg);
                    continue;
                }
            }

            else {
                const std::string msg =
                    "[UrdfLoader] " + ctx + ": parent link not declared: '" + parentName + "'.";
                if (opt_.strict) throw std::runtime_error(msg);
                warn(rep, msg);
                continue;
            }
        }

        if (!model.hasLink(childName)) {
            if (opt_.autoCreateMissingLinks) {
                try {
                    model.addLink(childName);
                    warn(rep, "[UrdfLoader] " + ctx + ": auto-created missing child link '" + childName + "'.");
                }
                catch (const std::exception& e) {
                    const std::string msg =
                        "[UrdfLoader] " + ctx + ": cannot auto-create child link '" + childName + "': " + e.what();
                    if (opt_.strict) throw;
                    warn(rep, msg);
                    continue;
                }
            }
            else {
                const std::string msg =
                    "[UrdfLoader] " + ctx + ": child link not declared: '" + childName + "'.";
                if (opt_.strict) throw std::runtime_error(msg);
                warn(rep, msg);
                continue;
            }
        }

        KinematicModel::Joint J;
        J.name = name;
        J.type = type;

        try {
            J.parentLink = model.linkId(parentName);
            J.childLink = model.linkId(childName);
        }

        catch (const std::exception& e) {
            const std::string msg =
                "[UrdfLoader] " + ctx + ": cannot resolve link ids: " + e.what();
            if (opt_.strict) throw;
            warn(rep, msg);
            continue;
        }

        J.origin = parseOrigin(jointEl, opt_.strict, rep, ctx);
        J.axis = parseAxis(jointEl, opt_.normalizeAxis, opt_.strict, rep, ctx);

        if (isContinuous) {
            J.limit.hasLimits = false;
            J.limit.lower = 0.0;
            J.limit.upper = 0.0;

            const tinyxml2::XMLElement* limitEl = optionalChild(jointEl, "limit");
            if (limitEl) {
                const char* lowerStr = limitEl->Attribute("lower");
                const char* upperStr = limitEl->Attribute("upper");

                if (lowerStr && upperStr && lowerStr[0] != '\0' && upperStr[0] != '\0') {
                    try {
                        const double lo = std::stod(lowerStr);
                        const double up = std::stod(upperStr);
                        if (lo > up) {
                            const std::string msg =
                                "[UrdfLoader] " + ctx + ": invalid <limit> for continuous joint (lower > upper).";
                            if (opt_.strict) throw std::runtime_error(msg);
                            warn(rep, msg);
                        }
                        else {
                            J.limit.hasLimits = true;
                            J.limit.lower = lo;
                            J.limit.upper = up;
                        }
                    }
                    catch (const std::exception&) {
                        const std::string msg =
                            "[UrdfLoader] " + ctx + ": cannot parse continuous <limit lower/upper>; ignoring limits.";
                        if (opt_.strict) throw std::runtime_error(msg);
                        warn(rep, msg);
                    }
                }
            }
        }
        else {
            parseLimit(jointEl, J.type, J.limit, opt_.strict, rep, ctx);
        }

        try {
            model.addJoint(J);
        }

        catch (const std::exception& e) {
            const std::string msg =
                "[UrdfLoader] " + ctx + ": cannot add joint: " + e.what();

            if (opt_.strict) throw;
            warn(rep, msg);
            continue;
        }
    }
}

void UrdfLoader::chooseAndSetRoot(KinematicModel& model, Report* rep) const
{
    if (model.linkCount() == 0) {
        const std::string msg = "[UrdfLoader] chooseAndSetRoot: model has no links.";
        
        if (opt_.strict)
            throw std::runtime_error(msg);
        
        warn(rep, msg);
        return;
    }

    if (opt_.rootLinkOverride.has_value()) {
        const std::string& rootName = opt_.rootLinkOverride.value();

        if (!model.hasLink(rootName)) {
            const std::string msg =
                "[UrdfLoader] chooseAndSetRoot: rootLinkOverride '" + rootName + "' not found in model.";

            if (opt_.strict) {
                throw std::runtime_error(msg);
            }

            warn(rep, msg);
        }

        else {
            model.setRoot(model.linkId(rootName));
            if (rep) rep->rootLinkName = rootName;
            return;
        }
    }

    std::vector<KinematicModel::LinkId> roots;
    roots.reserve(model.linkCount());

    for (std::size_t i = 0; i < model.linkCount(); ++i) {
        if (!model.parentJoint(i).has_value()) {
            roots.push_back(i);
        }
    }

    if (roots.empty()) {
        const std::string msg =
            "[UrdfLoader] chooseAndSetRoot: cannot determine root (no link without parent joint). "
            "Model may contain a cycle or invalid structure.";

        if (opt_.strict)
            throw std::runtime_error(msg);

        warn(rep, msg);

        model.setRoot(0);
        if (rep) rep->rootLinkName = model.link(0).name;
        return;
    }

    if (roots.size() > 1)
        warn(rep, "[UrdfLoader] chooseAndSetRoot: multiple root candidates found; choosing the first one.");

    model.setRoot(roots[0]);
    if (rep) rep->rootLinkName = model.link(roots[0]).name;
}

void UrdfLoader::fillStats(const KinematicModel& model, Report* rep) const
{
    if (!rep)
        return;

    rep->links = model.linkCount();
    rep->joints = model.jointCount();
    rep->activeJoints = model.activeJointCount();

    if (model.hasRoot()) {
        rep->rootLinkName = model.link(model.root()).name;
    }

    else {
        rep->rootLinkName.clear();
    }
}

KinematicModel UrdfLoader::loadFromFile(const std::string& urdfPath, Report* outReport) const
{
    if (outReport)
        *outReport = Report{};

    tinyxml2::XMLDocument doc;
    const tinyxml2::XMLError rc = doc.LoadFile(urdfPath.c_str());

    if (rc != tinyxml2::XML_SUCCESS) {
        const std::string err = doc.ErrorStr() ? std::string(doc.ErrorStr()) : std::string("unknown XML error");
        
        throw std::runtime_error("[UrdfLoader] Cannot load URDF file '" + urdfPath + "': " + err);
    }

    const tinyxml2::XMLElement* robotEl = findRobotElement(doc);

    if (outReport)
        outReport->robotName = readRobotName(robotEl);

    KinematicModel model;
    parseLinks(robotEl, model, outReport);
    parseJoints(robotEl, model, outReport);

    chooseAndSetRoot(model, outReport);

    model.validate();

    fillStats(model, outReport);
    return model;
}

KinematicModel UrdfLoader::loadFromString(const std::string& urdfXml, Report* outReport) const
{
    if (outReport)
        *outReport = Report{};

    tinyxml2::XMLDocument doc;
    const tinyxml2::XMLError rc = doc.Parse(urdfXml.c_str(), urdfXml.size());
    
    if (rc != tinyxml2::XML_SUCCESS) {
        const std::string err = doc.ErrorStr() ? std::string(doc.ErrorStr()) : std::string("unknown XML error");
        
        throw std::runtime_error("[UrdfLoader] Cannot parse URDF XML string: " + err);
    }

    const tinyxml2::XMLElement* robotEl = findRobotElement(doc);

    if (outReport)
        outReport->robotName = readRobotName(robotEl);

    KinematicModel model;
    parseLinks(robotEl, model, outReport);
    parseJoints(robotEl, model, outReport);

    chooseAndSetRoot(model, outReport);

    model.validate();

    fillStats(model, outReport);
    return model;
}
