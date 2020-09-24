
#include <sample_nodelet/class_test.h>
#include <iostream>


template <>
void DummyClass<int>::print() {
    std::cout << "You're using int and the object is:" << this->in_obj_ << std::endl;
};


template <>
void DummyClass<char>::print() {
    std::cout << "You're using char and the object is:" << this->in_obj_ << std::endl;
};