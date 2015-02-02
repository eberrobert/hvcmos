#include "worker.h"
#include "func.h"
#include "plot.h"
#include "geniobase.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <QTimer>



// Constructor
Worker::Worker()
{
    this->stop = false;
    this->running = false;
    this->timerRunning = false;

}

Worker::Worker(GenioBase* genio, QextSerialPort* kecom, globalconfig gconf, pixelconfig H35pixel, PCBconfig* pcbconf)
{
    this->stop = false;
    this->genio = genio;
    Funkcije::SetExternalGenio(genio);
    this->H35pixel = H35pixel;
    this->gconf = gconf;
    this->pcbconf = pcbconf;
    this->running = false;
    this->kecom = kecom;
    this->wait = false;
    this->readings = 1;
    this->Th1Readings = 0;
}

// Destructor
Worker::~Worker()
{
}

void Worker::quit()
{
    emit finished();
}

void Worker::StopWork()
{
    this->stop = true;
}

void Worker::DoSCurve1(int pixel, double startvoltage, double th1, double th2)
{
    std::stringstream pixelstring;
    pixelstring << "Pixel " << pixel;
    emit NewPlotCurve(QString::fromStdString(pixelstring.str()));

    double step = 0.001;
    for(double inj=startvoltage; inj > 0; inj -= step)
    {
        // Settings
        this->Set3DACs(th1, th2, inj);
        this->LoadDACPCB();
        // Read
        double effi = Efficiency(1);
        // Signals
        emit AddPlotData(inj, effi);
        emit resultready(inj, effi);
    }

    emit SCurvefinished();
    if(!running)
        emit ready();

}

void Worker::DoAllSCurves(double startvoltage, double th1, double th2)
{
    this->running = true;
    for(int i=0; i<H35pixel.size(); i++)
    {
        if(!this->stop)
        {
            SetSCurveConfig(i);
            ConfigChip();
            emit this->UpdateProgress(i);

            this->DoSCurve1(i, startvoltage, th1, th2);
        }
    }
    this->running = false;
    emit ready();
}

void Worker::SetSCurveConfig(int pixel)
{
    gconf.ABEn = true;
    gconf.CompOffB = true;
    gconf.CompOffNorm = false;
    gconf.EnLowPass = true;
    H35pixel.SetAnaInj(false);
    H35pixel.SetDigInjEn(false);
    H35pixel.SetLd(false);
    H35pixel.SetHBEn(false);
    // Injection into 1 pixel and Hitbus out.
    H35pixel.SetHBEn(pixel, true);
    H35pixel.SetAnaInj(pixel,true);
}

void Worker::ConfigChip()
{
    SetPattern(0x00);
    //16 global control bits (GCBits)
    bool GCBits[16]; //Global Control Bits
    int TDAC[3];     // Use this to read in from UI. Convert to bool later.

    TDAC[0] = 0;// TDAC 1
    TDAC[1] = 0; // TDAC 2
    TDAC[2] = 0; // TDAC 3
    for(int i=0; i<3; i++)
    {
        if(TDAC[i] > 15 || TDAC[i] < 0) // Check if integer is in 4bit range.
            TDAC[i] = 0;
    }

    // Is this corrrect with MSB and order of bits?
    //TDAC2
    GCBits[0] = 0x08 & TDAC[1];   //TDAC2(0:3) // MSB
    GCBits[1] = 0x04 & TDAC[1];   //TDAC2(0:3)
    GCBits[2] = 0x02 & TDAC[1];   //TDAC2(0:3)
    GCBits[3] = 0x01 & TDAC[1];   //TDAC2(0:3)
    //TDAC 1
    GCBits[4] = 0x08 & TDAC[0];   //TDAC1(0:3) // MSB
    GCBits[5] = 0x04 & TDAC[0];   //TDAC1(0:3)
    GCBits[6] = 0x02 & TDAC[0];   //TDAC1(0:3)
    GCBits[7] = 0x01 & TDAC[0];   //TDAC1(0:3)
    //TDAC 3
    GCBits[12] = 0x08 & TDAC[2];   //TDAC3(0:3) // MSB
    GCBits[13] = 0x04 & TDAC[2];   //TDAC3(0:3)
    GCBits[14] = 0x02 & TDAC[2];   //TDAC3(0:3)
    GCBits[15] = 0x01 & TDAC[2];   //TDAC3(0:3)


    GCBits[8] = gconf.ABEn; // ABEnB

    GCBits[9] = gconf.CompOffNorm; //CompOffBNormal

    GCBits[10] = gconf.CompOffB; //CompOffB

    GCBits[11] = gconf.EnLowPass; //EnLowPass

    //Write configuration into Chip
    // Start from 289 down to 0
    // Send 14 bias DACs (6bit and one spare) with SendDAC in class Funkcije. Bits 289 down to 192
    //
    std::cout << "|289 down to 192>> ";
    for(int i=13; i>=0; i--)
    {
        SendDAC(gconf.GetbDAC(i), gconf.GetSpare(i)); //SendDac(int, bool Sparebit)
        std::cout << "'";
    }
    std::cout << std::endl << "|191 down to 176>> ";
    // Send 16 control bits. 191 down to 176.
    for(int i=15;i>=0; i--)
    {
        SendBit(GCBits[i]);
    }
    std::cout << std::endl << "|175 down to 0  >> ";
    // Send configuration for 22 column pairs. Bits 175 down to 0
    for(int i=22; i>0; i--)
    {
        for(int RO=2; RO>0; RO--)
            SendBit(H35pixel.GetBits(RO,i).AnaInj); // AnaInj is last 2 bits
        for(int RO=2; RO>0; RO--)
            SendBit(H35pixel.GetBits(RO,i).Ld); // Ld
        for(int RO=2; RO>0; RO--)
            SendBit(H35pixel.GetBits(RO,i).HBEn); // HBEn
        for(int RO=2; RO>0; RO--)
            SendBit(H35pixel.GetBits(RO,i).DigInjEn); //DigInjEn is first 2 bits
        std::cout << "'";
    }
    std::cout << std::endl;
    LoadConf();
    SendBuffer();
}

void Worker::ScanTWDown(double th1, double th2)
{
    this->running = true;
    //Scan for pixel 1
    this->SetSCurveConfig(0);

    std::ofstream file;
    file.open("ScanTWDown.txt");
    std::vector<double> y;
    std::vector<int> TW;
    std::vector<double> inj;
    double delay;

    double startvoltage = 3.;
    for(int i=10; i<65; i+=1)
    {
        TW.push_back(i);
        gconf.SetbDAC(3,i);
        this->ConfigChip();
        std::stringstream dacvalue;
        dacvalue << "TWDown " << i;
        emit NewPlotCurve(QString::fromStdString(dacvalue.str()));
        for(double k=startvoltage; k > 0.1; k-=0.5)
        {
            this->Set3DACs(th1, th2, k);
            this->LoadDACPCB();
            delay = this->MeasureDelay();
            std::cout << delay << std::endl;
            y.push_back(delay);
            emit AddPlotData(k, delay);
        }
        std::cout << std::endl;
    }
    for(double k=startvoltage; k > 0.1; k-=0.1)
    {
        inj.push_back(k);

    }
    /*
    for(int k=0; k < inj.size(); k++)
    {
        if(k == 0)
        {
            file << "# ";
            for(int i=0; i<TW.size(); i++)
            {
                file << " " << TW[i];
            }
            file << "\n";
        }
        file << inj[k] << " ";
        for(int i=0; i<TW.size(); i++)
        {
            file << " " << y[i*TW.size()+k] ; // does not work....
        }
        file << "\n";
    }
*/

    file.close();
    this->running = false;
    emit ready();
}


void Worker::ScanTh2(int pixel, double th1)
{
    this->running = true;
    this->SetSCurveConfig(pixel);
    double inj1 = 1;
    double inj2 = 0.5;
    double delay = 0;
    std::stringstream pixelstring;
    pixelstring << "Pixel " << pixel;
    emit NewPlotCurve(QString::fromStdString(pixelstring.str()));
    for(double th2=th1; th2 > 0.2; th2 -= 0.1)
    {
        // Injection 1
        this->Set3DACs(th1, th2, inj1);
        this->LoadDACPCB();
        delay = this->MeasureDelay(10);

        // Injection 2
        this->Set3DACs(th1, th2, inj2);
        this->LoadDACPCB();
        delay -= this->MeasureDelay(10);
        emit AddPlotData(th2, delay);
    }
    this->running = false;
    emit ready();
}

// -----------------------------------------------------------
// COM Communication
// -----------------------------------------------------------

void Worker::SetReadings(int readings)
{
    this->readings = readings;
    std::cout << readings << std::endl;
}

void Worker::SetTh1Readings(int steps)
{
    this->Th1Readings = steps;
}

void Worker::readCOMData()
{
    this->readCOMData(this->readings);
}

void Worker::readCOMData(int readings)
{
    QByteArray data;
    if(kecom->canReadLine())
    {
    //std::cout << "line ready" << std::endl;
    data = kecom->readAll();

    //std::cout << data.size() << " data" << std::endl;
    QString qdata;
    qdata.append(data);
    std::string sdata;
    sdata = qdata.toStdString();
    //std::cout << sdata << std::endl;
    std::vector<double> reading;
    reading.resize(readings);
    size_t pos = 0;
    size_t found = 0;
    double average = 0;
    for(unsigned int i=0; i<reading.size(); i++)
    {
        found = sdata.find(",",pos);
        std::stringstream streamdata;
        streamdata << sdata.substr(pos,found-pos);
        pos = found +1;
        streamdata >> reading[i];
        std::cout << reading[i] << std::endl;
        average += reading[i];
    }
    average /=reading.size();
    emit COMdataReady(average);
    if(this->Th1Readings > 0)
    {
        this->Th1Readings--;
        emit CurrentReading(average);
    }
    else
    {
        emit ready(); 
    }
    }// if can read line
}

void Worker::GetPixelAddress()
{
    Funkcije::GetPixelAddress();
}



// ----------------------------------------------------------------
// This is the timer
// ----------------------------------------------------------------
void Worker::on_timer_update()
{
    if(this->timerRunning) // Pattern generation as long as checkbox is active.
    {
        StartPattern();
        SendBuffer();
        //std::stringstream counterstate;
        //counterstate << funcc->ReadCounterState();
        //logit("Hitbus Counter: " + counterstate.str());

    }
    if(this->TimerReadPixel)
    {
        Funkcije::GetPixelAddress();
    }
}

void Worker::StartTimer()
{
    this->timerRunning = true;
}

void Worker::StopTimer()
{
    this->timerRunning = false;
}

void Worker::StartTimerReadPixel()
{
    this->TimerReadPixel = true;
}

void Worker::StopTimerReadPixel()
{
    this->TimerReadPixel = false;
}

void Worker::EnableDigitalClock(bool enable)
{
    Funkcije::EnableDigitalClock(enable);
}

void Worker::SetDigPixDelay(int delay)
{
    Funkcije::SetDigpixDelay(delay);
    Funkcije::SendBuffer();

}

void Worker::SetDigPixClockdiv(int div)
{
    Funkcije::SetDigpixClockdiv(div);
    Funkcije::SendBuffer();
            std::cout << "changed clockdiv" << std::endl;
}

void Worker::FindLowestTh1(int pixel,bool flagSpare1)
{
    //std::cout << "Worker: Injection Value: " << pcbconf->GetInj() << "\n";

    gconf.ABEn = true;
    gconf.CompOffB = true;
    gconf.CompOffNorm = false;
    gconf.EnLowPass = true;
    // Set TDAC
    gconf.SetSpare(1, flagSpare1);

    H35pixel.SetAnaInj(false);
    H35pixel.SetDigInjEn(false);
    H35pixel.SetHBEn(false);
    H35pixel.SetLd(false);
    H35pixel.SetAnaInj(pixel, true);
    H35pixel.SetHBEn(pixel, true);
    this->ConfigChip();

    int Injections = 128;
    int counterstate = 0;
    double ThStep = 0.01;
    int Iterations = 0;
    int MaxIterations = 200;
    //double Threshold = 1.90;
    double Threshold = 1.90;
    double bestTh = 0.0;
    bool notfound = true;
    bool follower = false;
    int k;
    InitPatternHitbus(Injections);

    while (notfound)
    {
        Iterations++;
        if (Iterations == MaxIterations)
            break;

        Set3DACs(Threshold, pcbconf->GetTh2(), pcbconf->GetInj());
        LoadDACPCB();
        Threshold = Threshold + ThStep;
        InitCounter();
        StartPattern();
        SendBuffer();
        sleep(50);
        counterstate = ReadCounterState();
        if (counterstate <= Injections)
        {
            if (follower)
            {
                follower = false;
                if (ThStep == 0.1)
                {
                    Threshold = Threshold - 0.2;
                    ThStep = 0.01;
                }else if (ThStep == 0.01)
                {
                    Threshold = Threshold - 0.02;
                    ThStep = 0.001;
                }else if (counterstate == Injections)
                {
                    bestTh = Threshold - ThStep;
                    notfound = false;
                }

            } else
                follower = true;

        }
        //std::stringstream counterstate1;
        //std::stringstream counterstate2;
        //counterstate1 << ReadCounterState();
        //counterstate2 << Threshold;
        //logit("Threshold: " + counterstate2.str() + " Counter: " + counterstate1.str());
        std::cout << "Threshold :" << Threshold << " Counter :" << counterstate << std::endl;
    }
    Threshold = bestTh;

    if(!notfound)
    {
        notfound = true;
        while (notfound)
        {
            counterstate = 0;
            for (k = 0; k<10; k++)
            {
                Set3DACs(Threshold, pcbconf->GetTh2(), pcbconf->GetInj());
                LoadDACPCB();
                InitCounter();
                StartPattern();
                SendBuffer();
                sleep(50);
                counterstate = counterstate + ReadCounterState();
            }
            if (counterstate == 1280)
            {
                notfound = false;
                bestTh = Threshold;
            }
            else
                Threshold = Threshold + 0.001;
        }
    }

    std::stringstream counterstate1;
    counterstate1 << bestTh;
    QString blabla;

    if(notfound)
        blabla = "No Threshold found!";
    else
        blabla = "Best Threshold found: " + QString::fromStdString(counterstate1.str());
        
    emit Logit(blabla);

    emit ready();
}
