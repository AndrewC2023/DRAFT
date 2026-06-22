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

namespace DRAFT::Simulation
{

    class DraftSim
    {
        public:
            DraftSim();
            ~DraftSim();

            void Step();
            void Run();
        private:
            // List of simulation objects getting called in each step
            std::vector<Objects::DraftSimObject> draftSimObjects;

            // Internal Values
            double simTime_ = 10;
            double initialTime_ = 0;
            double stepSize_ = 0.001;

            // flags
            bool isRunning_;
            bool paused_;
            bool loggingEnabled_;

        

    };
}

#endif // DRAFTSIM_H