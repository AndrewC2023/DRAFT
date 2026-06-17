/*
 * Author: Andrew Campbell
 * Date: 06-15-2026
 */

#ifndef DRAFTVEHICLE_H
#define DRAFTVEHICLE_H

#include "DraftSimObject.hpp"
#include "src/Dynamics/IDynamics.hpp"

namespace DRAFT::Simulation::Objects
{
    class DraftVehicle : DraftSimObject
    {
        public:
            DraftVehicle();
            ~DraftVehicle() = default;
            
        private:
            // need dynamics
            // likely need some sort of class that idicates say the vehicle size
            // in future we may also want the actuation stuff here, likely then need actuation stuff in the dynamics folder
            // Maybe need stuff here to indicate how the vehicle would respond to certain sensor
            // also may want in a vehicle for it to have its percep

    };
}

#endif // DRAFTVEHICLE_H