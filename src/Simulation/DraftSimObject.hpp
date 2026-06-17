/*
 * Author: Andrew Campbell
 * Date: 06-15-2026
 */

#ifndef DRAFTSIMOBJECT_H
#define DRAFTSIMOBJECT_H

// STD
#include <cstdint>

// maybe string, TODO: implement what is returned or caught when running step so the parent sim knows what extra actions to do.
namespace DRAFT::Simulation::Objects
{
    class DraftSimObject
    {
        public:
            explicit DraftSimObject(std::uint64_t id) : id_(id) {};
            virtual ~DraftSimObject() = default;
            
            virtual void Step(double deltaTime) = 0; // Pure virtual function to be implemented by derived classes

            std::uint64_t getID_() const noexcept { return id_;};

        private:
            const std::uint64_t id_;
    };
}

#endif // DRAFTSIMOBJECT_H