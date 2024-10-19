/*
 * Author: Andrew Campbell
 * Date: 10-17-2024
 */

#ifndef VARIABLE_H
#define VARIABLE_H

#include <string>

namespace Math
{
    class Variable
    {
        public:
        Variable(std::string Name){name = Name;};
        ~Variable() = default;
        
        std::string name;
    };
}

#endif // VARIABLE_H