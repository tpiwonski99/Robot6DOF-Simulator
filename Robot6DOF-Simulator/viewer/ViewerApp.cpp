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