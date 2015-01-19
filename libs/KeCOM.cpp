#include <QByteArray>
#include <QString>
#include "KeCOM.h"
#include <stdlib.h>
#include <sstream>
/*reset = "*RST",
arm = "ARM:SOUR IMM",
armcount = "ARM:COUN 1",
trigger = "TRIG:SOUR IMM",
triggercount = "TRIG:COUN 1",
read = "READ?",
form = "FORM:ELEM READ",
idn = "*IDN?",
zerocheckoff = "SYST:ZCH OFF",
zerocorrectoff = "SYST:ZCOR OFF"
*/

QByteArray Ke6485::CreateCommand(std::string command)
{
    QByteArray data;
    for(unsigned int i=0; i<command.size(); i++)
    {
        data.push_back(command.at(i));
    }
#if defined(Q_OS_WIN)
    data.push_back('\n');
#endif
#if defined(Q_OS_LINUX)
    data.push_back("\n");
#endif
    return data;
}

QByteArray Ke6485::reset()
{
    return CreateCommand("*RST");
}

QByteArray Ke6485::arm()
{
    return CreateCommand("ARM:SOUR IMM");
}

QByteArray Ke6485::armcount(int armcount)
{
    std::stringstream command;
    command << "ARM:COUN " << armcount;
    return CreateCommand(command.str());
}

QByteArray Ke6485::form()
{
    return CreateCommand("FORM:ELEM READ");
}

QByteArray Ke6485::idn()
{
    return CreateCommand("*IDN?");
}

QByteArray Ke6485::trigger()
{
    return CreateCommand("TRIG:SOUR IMM");
}

QByteArray Ke6485::triggercount(int trigger)
{
    std::stringstream command;
    command << "TRIG:COUN " << trigger;
    return CreateCommand(command.str());
}

QByteArray Ke6485::zerocheck(bool on)
{
    std::string command = "SYST:ZCH ";
    if(on)
        command += "ON";
    else
        command += "OFF";
    return CreateCommand(command);
}

QByteArray Ke6485::zerocorrect(bool on)
{
    std::string command = "SYST:ZCOR ";
    if(on)
        command += "ON";
    else
        command += "OFF";
    return CreateCommand(command);
}

QByteArray Ke6485::read()
{
    return CreateCommand("READ?");
}
