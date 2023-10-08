#include "assignment2.hpp"
#include <string>
#include <vector>
#include <array>
#include <typeinfo>

//Konstandid (const, constexpr, muutujad)
bool RG::Homework2::assignment2()
{
    using namespace std::literals;
    int i{ 3 };
    // Looge const int muutuja ci väärtusega 5
    const int ci{5};

    // Looge constexpr int muutuja cxpi väärtusega 4
    constexpr int cxpi{4};

    // Looge const int pointer muutuja cip väärtusega ci
    const int* cip = &ci;

    // Looge int pointer const muutuja ipc väärtusega i
    int ipc = i;
    int* const ipc = &i;

    // Looge const int pointer const muutuja cipc väärtusega ci
    const int* const cipc = &ci;


    // Kontroll, et muutujad on õiget tüüpi ja väärtusega
    return (typeid(ci) == typeid(const int))
    && (ci == 5)
    && (typeid(cxpi) == typeid(const int))
    && (cxpi == 4)
    && (typeid(cip) == typeid(const int *))
    && (*cip == ci) 
    && (typeid(ipc) == typeid(int * const))
    && (*ipc == i)
    && (typeid(cipc) == typeid(const int * const))
    && (*cipc == ci);
}

