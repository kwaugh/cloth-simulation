#include "simulation.h"
#include "viewer_wrapper.h"
#include <thread>
#include <mutex>

using namespace std;

void doSimulation(Simulation* sim) {
    while (true)
        sim->takeSimulationStep();
}

int main() {
    mutex renderLock;
    Simulation *sim = new Simulation(renderLock);
    thread simThread(doSimulation, sim);
    ViewerWrapper viewer(sim, &renderLock);
    viewer.start();
}
