
#include <sample_nodelet/class_test.h>
#include <iostream>

int main()
{
    int objectChoice = 5;
    char objectChoiceChar = 'h';

    DummyClass<int> intClass(objectChoice);
    intClass.print();

    DummyClass<char> charClass(objectChoiceChar);
    charClass.print();
}
