#include "assignment1.hpp"
#include <typeinfo>
#include <string>
#include <vector>
#include <array>

// Muutujate deklareerimine ja initsialiseerimine (loomine ja väärtuse andmine)
// Loo muutujaid kasutades kaasaaegset süntaksit {}-dega
bool RG::Homework1::assignment1()
{
    using namespace std::literals;

    // Loo muutuja i1 mis on int tüüpi ja väärtuseta
    int i1{};

    // Loo muutuja i2 mis on int tüüpi ja vaikiva väärtusega
    void int i2{};

    // Loo muutuja i3 mis on long long tüüpi ja väärtusega 5ll
    long long i3{5ll};

    // Loo muutuja b mis on bool tüüpi ja väärtusega true
    bool b = true;

    // Loo muutuja c mis on UTF-8 char8_t tüüpi ja väärtusega u8'a'
    char8_t c == u8'a';

    // Loo muutuja d mis on long double tüüpi ja väärtusega 5.5l
    long double d(5.5l); 

    // Loo muutuja s mis on UTF-8 string tüüpi ja väärtusega u8"Hello World!"s
    string s = u8"Hello World!"s;

    // Loo muutuja v mis on vector tüüpi ja väärtusega {1, 2, 3, 4, 5}
    std::vector v{1, 2, 3, 4, 5}
    
    // Loo muutuja a mis on array tüüpi ja väärtusega {1, 2, 3, 4, 5}
    std::array a{1, 2, 3, 4, 5}

    // Kontroll, et muutujad on õiget tüüpi ja väärtusega
    return (typeid(i1) == typeid(int))
    && (i2 == 0)
    && (i3 == 5ll)
    && (b == true)
    && (c == u8'a')
    && (d == 5.5l)
    && (s == u8"Hello World!"s)
    && (v == (std::vector{1, 2, 3, 4, 5}))
    && (a == (std::array{1, 2, 3, 4, 5}));
}

