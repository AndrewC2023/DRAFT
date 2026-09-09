/*
 * Author: Andrew Campbell
 * Date: 06-13-2026
 */

#include "DraftSim.hpp"

namespace DRAFT::Simulation
{

    DraftSim::DraftSim()
    {
        // Constructor implementation
    }

    DraftSim::~DraftSim()
    {
        // Destructor implementation
    }

    void DraftSim::Run()
    {
        // Simulation run implementation
    }

    void DraftSim::Step()
    {
        for(int i = 0; i < draftSimObjects.size(); i++){
            draftSimObjects[i].Step(timeStepSize_);
        }
    }

    void DraftSim::Step(float timeStep)
    {
        for(int i = 0; i < draftSimObjects.size(); i++){
            draftSimObjects[i].Step(timeStep);
        }
    }
}