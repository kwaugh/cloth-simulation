#include "simulation.h"
#include "viewer_wrapper.h"
#include <thread>
#include <mutex>

using namespace std;

void doSimulation(Simulation* sim) {
    //while (true) {
    for (int i = 0; i < 1000; i++) {
        //if (i % 1000 == 0) cout << "stepCount: " << i << endl;
        sim->takeSimulationStep();
    }
    exit(1);
}

int main() {
    mutex renderLock;
    Simulation *sim = new Simulation(renderLock);
    thread simThread(doSimulation, sim);
    ViewerWrapper viewer(sim, &renderLock);
    viewer.start();
}
