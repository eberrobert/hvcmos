#ifndef SHIGH_H
#define SHIGH_H

#include<QThread>
#include "func.h"
#include "plot.h"


class shigh : public QThread, public Funkcije
{
    private:
        xydata run(double startvalue, int Measurements); // This starts the thread.
    public:
        shigh();
        ~shigh();
};

#endif // HIGHLEVEL_H
