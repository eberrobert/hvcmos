#ifndef KECOM_H
#define KECOM_H

#include <QByteArray>
#include <stdlib.h>

class Ke6485
{
    public:
    //Commands

        QByteArray reset();
        QByteArray arm();
        QByteArray armcount(int armcount);
        QByteArray trigger();
        QByteArray triggercount(int trigger);
        QByteArray form();
        QByteArray idn();
        QByteArray zerocheck(bool on);
        QByteArray zerocorrect(bool on);
        QByteArray read();
    private:
        QByteArray CreateCommand(std::string command);
};

#endif // KECOM_H
