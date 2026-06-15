/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef DRAFTSIM_H
#define DRAFTSIM_H

// Custom Types
#include <DraftSimObject.hpp>

// STD
#include <vector>

namespace DRAFT::Simulation
{

    class DraftSim
    {
        public:
            DraftSim();
            ~DraftSim();

            void step();
            void run();
        private:
            // List of simulation objects getting called in each step
            std::vector<DraftSimObject> draftSimObjects;

            // Internal Values
            double simTime = 10;
            double initialTime = 0;
            double stepSize = 0.001;

            // flags
            bool isRunning;
            bool paused;
            bool loggingEnabled;

        

    };
}

#endif // DRAFTSIM_H