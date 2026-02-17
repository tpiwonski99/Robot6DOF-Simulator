#pragma once 
#include <glfw3.h>

#include <memory>
#include <string>
#include <iostream>

#include "KinematicModel.hpp"
#include "RobotRunTime.hpp"
#include "UrdfLoader.hpp"

struct GLFWwindow;

class KinematicModel;
class RobotRunTime;

class ViewerApp {
public:

	struct Config {

		Config(int w, int h, const char* t, std::string path, bool strict) : width(w), height(h), title(t), urdfPath(path), strictUrdf(strict) {}

		int width;
		int height;
		const char* title;

		std::string urdfPath;
		bool strictUrdf;
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

	bool loadRobot_();

private:
	Config cfg_;

	GLFWwindow* window_ = nullptr;
	bool initialized_ = false;

	// ROBOT
	std::shared_ptr<KinematicModel> model_;
	std::shared_ptr<RobotRunTime> runtime_;
};