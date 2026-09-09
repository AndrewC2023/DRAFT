/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef DRAFTSIM_H
#define DRAFTSIM_H

// Custom Types
#include "DraftSimObject.hpp"

// STD
#include <vector>


/* TODO: need a logging system of some sort, Likely want a logging object that can write to
a file as well be passed to a sim object, this is so a logging object can be made for each item
and the parent sim and manage labels and what Is logged exetera. 

This system I guess would predate a real time visualization system? if we use pyqt to visualize
that will be different from parsing logs after a run

*/

namespace DRAFT::Simulation
{

    class DraftSim
    {
        public:
            DraftSim();
            ~DraftSim();

            void Step();
            void Step(float timeStep);
            void Run();
        private:
            // List of simulation objects getting called in each step
            std::vector<Objects::DraftSimObject> draftSimObjects;

            // Internal Values
            double globalSimTime_ = 0;
            double initialTime_ = 0;
            double timeStepSize_ = 0.001;

            // flags
            bool isRunning_;
            bool paused_;
            bool loggingEnabled_;

        

    };
}

#endif // DRAFTSIM_H