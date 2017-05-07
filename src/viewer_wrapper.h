#ifndef VIEWER_WRAPPER_H
#define VIEWER_WRAPPER_H
#include "simulation.h"
#include <mutex>

class ViewerWrapper {

    public:
        ViewerWrapper(Simulation *sim, mutex* renderLock) : sim_(sim), renderLock(renderLock) { }
	void start(); 
    private:
	Simulation *sim_;
        mutex* renderLock;
}; 
#endif
