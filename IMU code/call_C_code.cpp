/*/ C++ code
    extern "C" void f(int); // one way
    extern "C" {    // another way
        int g(double);
        double h();
    };
    void code(int i, double d)
    {
        f(i);
        int ii = g(d);
        double dd = h();
    }
*/
#ifndef SOMECODE_H_
#define SOMECODE_H_

 void foo();

 #endif
