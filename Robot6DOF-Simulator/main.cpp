#include "viewer/ViewerApp.hpp"

int main() {
    ViewerApp::Config config(1280, 720, "Robot6DOF Viewer");
    ViewerApp app(config);
    if (!app.init())
        return 1;

    app.run();
    return 0;
}

