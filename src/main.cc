#include "simulation.h"
#include "viewer_wrapper.h"

int main() {
    Simulation *sim = new Simulation();
    ViewerWrapper viewer(sim);
    viewer.start();
}
