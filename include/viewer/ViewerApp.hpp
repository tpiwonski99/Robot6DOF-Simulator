#pragma once 
#include <glfw3.h>

#include <memory>
#include <string>
#include <iostream>

#include "KinematicModel.hpp"
#include "RobotRunTime.hpp"

struct GLFWwindow;

class KinematicModel;
class RobotRunTime;

class ViewerApp {
public:

	struct Config {

		Config(int w, int h, const char* t) : width(w), height(h), title(t) {}

		int width;
		int height;
		const char* title;
	};

public:

	explicit ViewerApp(Config cfg);
	~ViewerApp();

	bool init();
	void run();
	void shutdown();

private:

	static void glfwErrorCallback_(int error, const char* description);
	static void keyCallback_(GLFWwindow* window, int key, int scancode, int action, int mods);

private:
	Config cfg_;

	GLFWwindow* window_ = nullptr;
	bool initialized_ = false;

	// ROBOT
	std::shared_ptr<KinematicModel> model_;
	std::shared_ptr<RobotRunTime> runtime_;
};