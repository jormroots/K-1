#include <iostream>
#include <cassert>
#include <functional>
#include <string>
#include "main.hpp"
#include "Assignment1/assignment1.hpp"
#include "Assignment2/assignment2.hpp"
#include "Assignment3/assignment3.hpp"

void test_assignment(std::string name, std::function<bool()> assignment){
    bool success{ assignment() };
    std::cout << name << ": " << (success ? "Success" : "Failure") << '\n';
    assert(success);
}

int main(){
    test_assignment("Assignment1", RG::Homework1::assignment1);
    test_assignment("Assignment2", RG::Homework1::assignment2);
    test_assignment("Assignment3", RG::Homework1::assignment3);

}
