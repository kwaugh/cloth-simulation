#include "viewer_wrapper.h"
#include "simulation.h"

#include <functional>

#include <igl/viewer/Viewer.h>
#include <igl/jet.h>

using namespace std;
using namespace Eigen;

// anonymous namespace to handle LibIGL viewer.
namespace {
    bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int mod,
            Simulation *sim) {
        switch (key) {
            case ' ':
                sim->paused = !sim->paused;
                break;
            case 'R':
                sim->reset();
                break;
        }
        return false;
    }

    bool init(igl::viewer::Viewer& viewer, Simulation *sim) {
        viewer.ngui->addWindow(Vector2i(220, 10), "Cloth Simulation");

        // Simulation environment variables.
        viewer.ngui->addGroup("Simulation Parameters");
        viewer.ngui->addVariable("Time Step",                  sim->timeStep);
        viewer.ngui->addVariable<bool>("Gravity Force",        sim->F_GRAV);
        viewer.ngui->addVariable<bool>("Stretch Force",        sim->F_STRETCH);
        viewer.ngui->addVariable<bool>("Shear Force",          sim->F_SHEAR);
        viewer.ngui->addVariable<bool>("Bend Force",           sim->F_BEND);
        viewer.ngui->addVariable<bool>("Collisions",           sim->COLLISIONS);
        viewer.ngui->addVariable<bool>("Visualize Collisions", sim->VIS_COLLISIONS);
        viewer.ngui->addVariable<int>("Cloth Version",         sim->vCloth);
        viewer.ngui->addVariable<double>("Cloth Scale",        sim->scale);
        viewer.ngui->addVariable<double>("Cloth Thickness",    sim->clothThickness);

        // Simulation controls..
        viewer.ngui->addGroup("Simulation Controls");
        viewer.ngui->addButton("Toggle Simulation",
                [sim](){ sim->paused = !sim->paused; });
        viewer.ngui->addButton("Reset Simulation",
                [sim](){ sim->reset(); });

        // Generate widgets.
        viewer.screen->performLayout();

        return false;
    }

    bool pre_draw(igl::viewer::Viewer& viewer, Simulation *sim, mutex* renderLock) {
        // Get the current mesh of the simulation.
        MatrixX3d V;
        MatrixX3i F;
        VectorXd C;
        renderLock->lock(); {
            sim->generate_libigl_geometry(V, F, C);
        } renderLock->unlock();

        // Update the viewer.
        viewer.data.clear();
        viewer.data.set_mesh(V, F);

        if (sim->VIS_COLLISIONS && sim->COLLISIONS && C.rows() > 0) {
            MatrixX3d C_jet;
            igl::jet(C, true, C_jet);
            viewer.data.set_colors(C_jet);
        }

        if (!sim->stepCount)
            viewer.core.align_camera_center(V, F);

        // Signal to render.
        glfwPostEmptyEvent();

        return false;
    }

} // end anonymous namespace to handle LibIGL viewer.

void ViewerWrapper::start() {
    igl::viewer::Viewer viewer;
    viewer.callback_key_down = bind(key_down,
            placeholders::_1,
            placeholders::_2,
            placeholders::_3,
            sim_);
    viewer.callback_init = bind(init, placeholders::_1, sim_);
    viewer.callback_pre_draw = bind(pre_draw, placeholders::_1, sim_, renderLock);

    viewer.launch();
}
