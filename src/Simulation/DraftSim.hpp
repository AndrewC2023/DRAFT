/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#ifndef DRAFTSIM_H
#define DRAFTSIM_H

namespace DRAFT::Simulation
{

    class DraftSim
    {
        public:
            DraftSim();
            ~DraftSim();

            void run();
        private:
            // List of simulation objects getting called in each step
            float draftSimObjects;// Placeholder for actual simulation objects

            // flags
            bool isRunning;
            bool paused;
            bool loggingEnabled;

        

    };
}

#endif // DRAFTSIM_H