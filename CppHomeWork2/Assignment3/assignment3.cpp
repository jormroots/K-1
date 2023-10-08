#include "assignment3.hpp"
#include <string>
#include <vector>
#include <array>
#include <typeinfo>
#include <iostream>

// Funktsioonid (functions)
// Looge funktsioon funk, mis ei tagasta midagi
// ja prindib (cout) "Hello World!" konsoolile
int funk(){
    return 0;
}



// Looge funktsioon funk5, mis tagastab int väärtuse 5
int funk5(){
    return 5;
}



// Looge funktsiooni sum deklaratsioon, 
// mis tagastab kahe int väärtuse summa
int sum( int a, int b);



// Looge funktsioon sum, mis tagastab kahe int väärtuse summa
int sum(int a, int b){
    return a + b;
}



// Looge funktsioon sum, mis tagastab kahe double väärtuse summa
double sum(double a, double b){
    return a + b;
}



// Looge funktsioon increase, mis suurendab int väärtust 1 võrra
int increase(int value){
    return value + 1;
}



// Looge funktsioon decrease, mis vähendab int väärtust 1 võrra
// kasutades pointerit
void decrease(int* value){
    (*value)--;
}



// Looge constexpr funktsioon square, mis tagastab int väärtuse ruudu
constexpr int square(int x){
    return x * x;
}



// Looge consteval funktsioon cube, mis tagastab int väärtuse kuupi
consteval int cube(int y){
    return y * y * y;
}



//Funktsioonide kutsumine
bool RG::Homework2::assignment3()
{
    using namespace std::literals;
    int i1{ 3 };
    int i2{ 8 };

    // Kutsuge funktsiooni funk
    int funk();

    // Kutsuge funktsiooni funk5 
    // ja salvestage tulemus muutujasse a1
    int a1{ funk5() };

    // Kutsuge funktsiooni sum argumentidega 3, 2 
    // ja salvestage tulemus muutujasse a2
    int a2{ sum(3, 2) };

    // Kutsuge funktsiooni sum argumentidega 5.4, 4.5 
    // ja salvestage tulemus muutujasse a3
    double a3{ sum(5.4, 4.5) };

    // Kutsuge funktsiooni increase argumentidega i1
    int increase(i1);

    // Kutsuge funktsiooni decrease argumentidega i2
    int decrease(i2);

    // Kutsuge funktsiooni square argumentidega 3
    // ja salvestage tulemus muutujasse a4
    int a4{ square(3) };

    // Kutsuge funktsiooni cube argumentidega 3
    // ja salvestage tulemus muutujasse a5
    int a5{ cube(3) };

    // Kontroll, et muutujad on õiget tüüpi ja väärtusega
    static_assert(square(5) == 25);
    return (typeid(&funk) == typeid(void(*)()))
    && (typeid(&funk5) == typeid(int(*)()))
    && (a1 == 5)
    && (a2 == 5)
    && (a3 == 9.9)
    && (typeid(&increase) == typeid(void(*)(int&)))
    && (i1 == 4)
    && (typeid(&decrease) == typeid(void(*)(int*)))
    && (i2 == 7)
    && (typeid(&square) == typeid(int(*)(int)))
    && (a4 == 9)
    && (typeid(&cube) == typeid(int(*)(int)))
    && (a5 == 27);
}

