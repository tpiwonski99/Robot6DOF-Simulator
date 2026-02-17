#include "viewer/ViewerApp.hpp"

ViewerApp::ViewerApp(Config cfg) : cfg_(cfg) {

}

ViewerApp::~ViewerApp() {
	shutdown();
}

bool ViewerApp::init() {

	glfwSetErrorCallback(ViewerApp::glfwErrorCallback_);
	if (!glfwInit()) {
		std::cerr << "[ViewerApp] glfwInit() failed\n";
		return false;
	}

	window_ = glfwCreateWindow(cfg_.width, cfg_.height, cfg_.title, NULL, NULL);

	if (!window_) {
		std::cerr << "[ViewerApp] glfwCreateWindow() failed\n";
		glfwTerminate();
		return false;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	glfwMakeContextCurrent(window_);
	glfwSetKeyCallback(window_, ViewerApp::keyCallback_);

	initialized_ = true;
	return true;
}

void ViewerApp::run() {
	if (!initialized_ || !window_) return;

	while (!glfwWindowShouldClose(window_)) {
		glfwPollEvents();
		glfwSwapBuffers(window_);
	}
}

void ViewerApp::shutdown() {
	if (window_) {
		glfwDestroyWindow(window_);
		window_ = nullptr;
	}

	if (initialized_) {
		glfwTerminate();
		initialized_ = false;
	}
}

void ViewerApp::glfwErrorCallback_(int error, const char* description) {
	std::cerr << "[GLFW ERROR] (" << error << ") " << description << "\n";
}

void ViewerApp::keyCallback_(GLFWwindow* window, int key, int scancode, int action, int mods) {
	(void)scancode; (void)mods;

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

bool ViewerApp::loadRobot_() {
	if (cfg_.urdfPath.empty()) {
		std::cerr << "[ViewerApp] loadRobot_: cfg_.urdfPath is empty.\n";
		return false;
	}

	try {
		UrdfLoader::Options opt;
		opt.strict = cfg_.strictUrdf;

		UrdfLoader loader(opt);

		UrdfLoader::Report rep;
		KinematicModel tmp = loader.loadFromFile(cfg_.urdfPath, &rep);

		tmp.validate();

		model_ = std::make_shared<KinematicModel>(std::move(tmp));

		runtime_ = std::make_unique<RobotRunTime>(model_);

		std::vector<double> q0(runtime_->dof(), 0.0);
		runtime_->setQ(q0);

		std::cout << "[URDF] robotName=" << rep.robotName
			<< " root=" << rep.rootLinkName
			<< " links=" << rep.links
			<< " joints=" << rep.joints
			<< " activeJoints=" << rep.activeJoints
			<< "\n";

		for (const auto& w : rep.warnings) {
			std::cout << "[URDF warning] " << w << "\n";
		}

		return true;
	}
	catch (const std::exception& e) {
		std::cerr << "[ViewerApp] loadRobot_ failed: " << e.what() << "\n";
		model_.reset();
		runtime_.reset();
		return false;
	}
}