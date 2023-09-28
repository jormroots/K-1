#include "assignment3.hpp"
#include <string>
#include <vector>
#include <array>

// Tehted muutujatega (+, -, *, /, %, ++, --, +=, -=, *=, /=, %=)
bool RG::Homework1::assignment3()
{
    using namespace std::literals;
    int i1{};
    long long i2{5ll};
    long double d{4.5l};
    char8_t c{u8'd'};
    std::u8string s{u8"Hello World!"s};
    std::vector v{1, 2, 3, 4, 5};
    std::array a{2, 3, 7, 5, 6};

    // Suurenda muutuja i1 väärtust 1 võrra kolmel erineval viisil
    i1 = i1 + 1;


    // Eemalda muutuja i2 väärtusest i1 väärtus
    i2 = i2 - i1;

    // Liida muutuja d väärtusele 0.3l ja jagada seejärel i1 väärtusega
    long double d = (d + 0.3l) / i1;

    // Liida c väärtusele i1 väärtus
    c = c + i1;

    // Liida s väärtusele c väärtus
    s = s + c;

    // Korruta (*=) muutuja v.at(1) väärtus v.at(2) väärtusega, mis on suurendatud ++-ga
    v.at(1) *= (v.at(2) + 1);

    // Liida muutujale a.at(2) väärtus v.at(3) väärtusega, siis korruta i1 väärtusega, 
    // seejärel lahuta i2 väärtusega, seejärel jaga v.at(4) väärtusega
    int result = (a.at(2) + v.at(3)) * i1 - i2 / v.at(4);
    
    // Kontroll, et muutujad on õiget tüüpi ja väärtusega
    return (i1 == 3)
    && (i2 == 8ll)
    && (d == 4.8l/3)
    && (c == u8'g')
    && (s == u8"Hello World!g"s)
    && (v == (std::vector{1, 8, 4, 4, 5}))
    && (a == (std::array{2, 3, 5, 5, 6}));
}

