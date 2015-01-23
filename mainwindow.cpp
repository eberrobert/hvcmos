#include "mainwindow.h"
#include "ui_mainwindow.h"

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <bitset>
#include <string>

#include <QTimer>
#include <QThread>

//Project specific
#include "libs/geniobase.h"
#include "libs/ftdi.h"
#include "libs/func.h"
#include "libs/config.h"
#include "libs/plot.h"
#include "libs/worker.h"
#include "libs/KeCOM.h"

//QCustomPlot
#include "libs/qcustomplot.h"

//QSerialExtPort
/* To use QExtSerialPort, add the following line to your project file (*.pro)
 * include(pathToPri/qextserialport.pri)
 */
#include "qextserialport-1.2rc/src/qextserialport.h"
#include "qextserialport-1.2rc/src/qextserialenumerator.h"

#if defined(__linux__)

#endif

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    kecom = NULL; // initalize pointer
    ui->setupUi(this);

    // For the serial port, list all devices present.
    QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();
    printf("List of ports:\n");
    for (int i = 0; i < ports.size(); i++)
    {
        printf("port name: %s\n", ports.at(i).portName.toLocal8Bit().constData());
        printf("friendly name: %s\n", ports.at(i).friendName.toLocal8Bit().constData());
        printf("physical name: %s\n", ports.at(i).physName.toLocal8Bit().constData());
        printf("enumerator name: %s\n", ports.at(i).enumName.toLocal8Bit().constData());
        printf("===================================\n\n");
    }
    // We have defined a default port "--" not to change the index while loading ports.
    // Otherwise, adding the first item will trigger the valueChanged Slot.
    // This will open the wrong COM port, if the first COM port is not the desired device
    int validports = 0;
    for(int i=0; i<ports.size(); i++)
    {
        std::stringstream comport;
        comport << ports[i].portName.toLocal8Bit().constData();
        if(comport.str() != "") // if the device doesn't have a port, don't list it.
        {
            this->ui->comboBox_3->addItem(QString::fromStdString(comport.str()),ports[i].portName.toLocal8Bit().constData());
            validports++;
        }
        std::stringstream enumport;
        enumport << ports[i].enumName.toLocal8Bit().constData();
        // Our device uses the FTDIBUS as description.
        if(enumport.str() == "FTDIBUS")
        {
            this->ui->comboBox_3->setCurrentIndex(validports);
        }
    }

    // QCustomPlot
    //customPlot = new QCustomPlot();
    plotcounter = 0;
    this->StopMeas = false;

    // Default configuration for H35 chip
    ui->hslide0->setValue(ui->bdac0->text().toInt());
    ui->hslide1->setValue(ui->bdac1->text().toInt());
    ui->hslide2->setValue(ui->bdac2->text().toInt());
    ui->hslide3->setValue(ui->bdac3->text().toInt());
    ui->hslide4->setValue(ui->bdac4->text().toInt());
    ui->hslide5->setValue(ui->bdac5->text().toInt());
    ui->hslide6->setValue(ui->bdac6->text().toInt());
    ui->hslide7->setValue(ui->bdac7->text().toInt());
    ui->hslide8->setValue(ui->bdac8->text().toInt());
    ui->hslide9->setValue(ui->bdac9->text().toInt());
    ui->hslide10->setValue(ui->bdac10->text().toInt());
    ui->hslide11->setValue(ui->bdac11->text().toInt());
    ui->hslide12->setValue(ui->bdac12->text().toInt());
    ui->hslide13->setValue(ui->bdac13->text().toInt());

    gconf.SetbDAC(0, ui->bdac0->text().toInt());
    gconf.SetbDAC(1, ui->bdac1->text().toInt());
    gconf.SetbDAC(2, ui->bdac2->text().toInt());
    gconf.SetbDAC(3, ui->bdac3->text().toInt());
    gconf.SetbDAC(4, ui->bdac4->text().toInt());
    gconf.SetbDAC(5, ui->bdac5->text().toInt());
    gconf.SetbDAC(6, ui->bdac6->text().toInt());
    gconf.SetbDAC(7, ui->bdac7->text().toInt());
    gconf.SetbDAC(8, ui->bdac8->text().toInt());
    gconf.SetbDAC(9, ui->bdac9->text().toInt());
    gconf.SetbDAC(10, ui->bdac10->text().toInt());
    gconf.SetbDAC(11, ui->bdac11->text().toInt());
    gconf.SetbDAC(12, ui->bdac12->text().toInt());
    gconf.SetbDAC(13, ui->bdac13->text().toInt());

    pcbconfig.SetInj(ui->pcbdac_edit->text().toDouble());
    pcbconfig.SetTh1(ui->pcbdac_edit_2->text().toDouble());
    pcbconfig.SetTh2(ui->pcbdac_edit_3->text().toDouble());

    gconf.ABEn = ui->gABEnB->text().toInt();
    gconf.CompOffB = ui->gCompOffB->text().toInt();
    gconf.CompOffNorm = ui->gCompOffNorm->text().toInt();
    gconf.EnLowPass = ui->gEnLowPass->text().toInt();

    ui->DACSpare0->setChecked(false);
    ui->DACSpare1->setChecked(false);
    ui->DACSpare2->setChecked(false);

    // Set the H35 pixel layout
    H35pixel.SetLayout(2,22);
    singlepixel activepixel;
    activepixel.AnaInj = 1;
    activepixel.DigInjEn = 0;
    activepixel.HBEn = 0;
    activepixel.Ld = 0;
    //All pixels
    H35pixel.SetConfig(0,1,0,0);
    //active pixel
    H35pixel.SetConfig(0, activepixel);

    ui->AnaInjText->setText(QString::fromStdString(H35pixel.GetPatternAnaInj()));
    ui->LdText->setText(QString::fromStdString(H35pixel.GetPatternLd()));
    ui->DigInjText->setText(QString::fromStdString(H35pixel.GetPatternDigInj()));
    ui->HBEnText->setText(QString::fromStdString(H35pixel.GetPatternHBEn()));



    ui->SCurveProgress->setValue(0);
    // by Robert Eber
    try
    {
        genio = new GenioBase();
        ftdi = new FTDI();

        int numDevs;
        char *BufPtrs[4];    // pointer to array of 3 pointers
        char Buffer1[64];    // buffer for description of first device
        char Buffer2[64];    // buffer for description of second device
        char Buffer3[64];

        // initialize the array of pointers
        BufPtrs[0] = Buffer1;
        BufPtrs[1] = Buffer2;
        BufPtrs[2] = Buffer3;
        BufPtrs[3] = NULL;    // last entry should be NULL
        //Try to get the right device
        FT_ListDevices(BufPtrs, &numDevs, FT_LIST_ALL|FT_OPEN_BY_SERIAL_NUMBER);
        std::cout << "Number of FTDI Devices: " << numDevs << std::endl;
        /*
            std::cout << Buffer1 << std::endl
                      << Buffer2 << std::endl
                      << Buffer3 << std::endl;
                      */
        //Buffers are 0 terminated strings...
        bool open = false;
        for(int i=0; i<numDevs; i++)
        {
            //Serial Numbers for this UXibo Board: 000012A and 000012B
            // Use Channel B for Communication
            std::stringstream serial;
            serial << BufPtrs[i];
            //std::cout << serial.str() << std::endl;

            if(serial.str().find_first_of("B") < serial.str().size())
            {
                open = ftdi->Open(i);
                std::cout << "Opened Device with serial " << serial.str() << " Nr. " << i << std::endl;
            }
        }

        if(!open)
            throw 2;
        else
            logit("Board successfully initialized.");
        genio->initializeFtdi(ftdi);
    }
    catch(int exception)
    {
         switch(exception)
         {
         case 2:
            logit("The Board could not be initialized.");
            break;
         default:
            logit("Unknown Error. This is bad.");
         }
    }

    // The COM Communication
    // Different Port names for windows and linux
#if defined(Q_OS_LINUX)
    //kecom = new QextSerialPort("/dev/ttyS0", QextSerialPort::EventDriven);
#endif


    if(kecom == NULL)
    {
        kecom = new QextSerialPort();
    }



    //The worker thread
    QThread* thread = new QThread();
    Worker* worker = new Worker(genio, kecom, gconf, H35pixel, &pcbconfig);
    worker->moveToThread(thread);

    // Connection of Signals and Slots
    connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    connect(worker, SIGNAL(resultready(double,double)), this, SLOT(HandleResults(double, double)));
    connect(this, SIGNAL(StartSCurve(int, double, double, double)), worker, SLOT(DoSCurve1(int, double,double,double)));
    connect(worker, SIGNAL(UpdateProgress(int)), this, SLOT(UpdateSCurveProgress(int)));
    connect(worker, SIGNAL(SCurvefinished()), this, SLOT(WriteSCurve()));
    connect(this, SIGNAL(StartSCurveAll(double,double,double)), worker, SLOT(DoAllSCurves(double,double,double)));
    connect(this, SIGNAL(StopWork()), worker, SLOT(StopWork()));
    connect(this, SIGNAL(ScanTWDown(double,double)), worker, SLOT(ScanTWDown(double,double)));
    connect(worker, SIGNAL(AddPlotData(double,double)), this, SLOT(AddPlotData(double,double)));
    connect(this, SIGNAL(ScanTh2(int,double)), worker, SLOT(ScanTh2(int,double)));
    connect(worker, SIGNAL(NewPlotCurve(QString)), this, SLOT(NewPlotCurve(QString)));
    connect(worker, SIGNAL(ready()), this, SLOT(ready()));
    //connect(this, SIGNAL(Th1Sweep(double)), worker, SLOT(Th1Sweep(double)));
    connect(kecom, SIGNAL(readyRead()), worker, SLOT(readCOMData()));
    connect(this, SIGNAL(SetReadings(int)), worker, SLOT(SetReadings(int)));
    connect(worker, SIGNAL(COMdataReady(double)), this, SLOT(COMdataReady(double)));
    connect(worker, SIGNAL(CurrentReading(double)), this, SLOT(CurrentReading(double)));
    connect(this, SIGNAL(Th1Readings(int)), worker, SLOT(SetTh1Readings(int)));
    connect(this, SIGNAL(GetPixelAddress()), worker, SLOT(GetPixelAddress()));
    connect(this, SIGNAL(StartTimer()), worker, SLOT(StartTimer()));
    connect(this, SIGNAL(StartTimerReadPixel()), worker, SLOT(StartTimerReadPixel()));
    connect(this, SIGNAL(StopTimer()), worker, SLOT(StopTimer()));
    connect(this, SIGNAL(StopTimerReadPixel()), worker, SLOT(StopTimerReadPixel()));
    connect(this, SIGNAL(EnableDigitalClock(bool)), worker, SLOT(EnableDigitalClock(bool)));
    connect(this, SIGNAL(SetDigPixDelay(int)), worker, SLOT(SetDigPixDelay(int)));
    connect(this, SIGNAL(SetDigPixClockDiv(int)), worker, SLOT(SetDigPixClockdiv(int)));
    connect(this, SIGNAL(FindLowestTh1(int, bool)), worker, SLOT(FindLowestTh1(int, bool)));
    connect(worker, SIGNAL(Logit(QString)), this, SLOT(Logit(QString)));

    thread->start();
    this->blocked = false;

    // QTimer
    // For continuous injection of pulses.
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), worker, SLOT(on_timer_update()));
    timer->start(1000);

}

MainWindow::~MainWindow()
{
    delete ui;
    delete genio;
    delete ftdi;
    delete kecom;
}

void MainWindow::logit(std::string logstream)
{
    std::cout << logstream << std::endl;
    QString buffer= QString::fromStdString(logstream);
    ui->Log->append(buffer);
}

void MainWindow::Logit(QString logger)
{
    logit(logger.toStdString());
}

void MainWindow::on_actionExit_triggered()
{
    MainWindow::close();
}

void MainWindow::on_actionHV_CMOS_GUI_triggered()
{

}

// -------------------------------------------
// SLOTS
// -------------------------------------------
void MainWindow::ready()
{
    this->blocked = false;
    this->on_ConfigReg_clicked();
    this->on_PCB_DAC_clicked();
    logit("Action finished.");
}

void MainWindow::on_ConfigReg_clicked()
{
    Funkcije* funcc = new Funkcije(this->genio);

    funcc->SetPattern(0x00);

    //Retrieve information about pixel setup from ui.

    //16 global control bits (GCBits)
    bool GCBits[16]; //Global Control Bits
    int TDAC[3];     // Use this to read in from UI. Convert to bool later.

    TDAC[0] = ui->gtdac1->text().toInt();// TDAC 1
    TDAC[1] = ui->gtdac2->text().toInt(); // TDAC 2
    TDAC[2] = ui->gtdac3->text().toInt(); // TDAC 3
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
        funcc->SendDAC(gconf.GetbDAC(i), gconf.GetSpare(i)); //SendDac(int, bool Sparebit)
        std::cout << "'";
    }
    std::cout << std::endl << "|191 down to 176>> ";
    // Send 16 control bits. 191 down to 176.
    for(int i=15;i>=0; i--)
    {
        funcc->SendBit(GCBits[i]);
    }
    std::cout << std::endl << "|175 down to 0  >> ";
    // Send configuration for 22 column pairs. Bits 175 down to 0
    for(int i=22; i>0; i--)
    {
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).AnaInj); // AnaInj is last 2 bits
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).Ld); // Ld
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).HBEn); // HBEn
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).DigInjEn); //DigInjEn is first 2 bits
        std::cout << "'";
    }
    std::cout << std::endl;
    //logit("AnaInjPattern:");
   // logit(H35pixel.GetPatternAnaInj());
    //logit("Ld Pattern:");
    //logit(H35pixel.GetPatternLd());
    //logit("HBEn Pattern:");
    //logit(H35pixel.GetPatternHBEn());
    //logit("DigInj Pattern:");
    //logit(H35pixel.GetPatternDigInj());

    // Set the load bit
    funcc->LoadConf();
    // Send Config
    try
    {
        if(!funcc->SendBuffer())
            throw 3;
        logit("Config written into Chip and loaded.");
    }
    catch(int exception)
    {
        switch(exception)
        {
            case 3: logit("Communication with FPGA failed.");
                break;
            default:
                logit("Unkown Error.");
        }
    }

    delete funcc;

}

// Configure DACs on the HV-Board PCB
// Uses register 0x43 (64+3)
void MainWindow::on_PCB_DAC_clicked()
{
    Funkcije* funcc = new Funkcije(genio);
    pcbconfig.SetInj(ui->pcbdac_edit->text().toDouble());
    pcbconfig.SetTh1(ui->pcbdac_edit_2->text().toDouble());
    pcbconfig.SetTh2(ui->pcbdac_edit_3->text().toDouble());

        // 3 DACs on the PCB
        funcc->Set3DACs(pcbconfig.GetTh1(),pcbconfig.GetTh2(),pcbconfig.GetInj());
        funcc->LoadDACPCB();
        try
        {
            if(!funcc->SendBuffer())
                throw 3;
        }
        catch(int exception)
        {
            switch(exception)
            {
                case 3: logit("Communication Error.");
                    break;
                default: logit("Unknown Error.");
            }
        }
        logit("Injection: " + ui->pcbdac_edit->text().toStdString() + "V, Th1: "
              + ui->pcbdac_edit_2->text().toStdString() + "V, Th2: "
              +  ui->pcbdac_edit_3->text().toStdString() + "V."
              );

    delete funcc;
}

// Configures and starts the Sequence with 128 cycles
//
void MainWindow::on_pushButton_3_clicked()
{
    if(!this->blocked)
    {
        Funkcije* funcc = new Funkcije(genio);

        //Write a class for the "Pattern"?
        funcc->InitPatternHitbus();
        funcc->StartPattern();
        funcc->SendBuffer();
        //funcc->ParkPattern(); // might be tight... smt like sleep should be here, but not sleep
        //funcc->SendBuffer();

        logit("Analog / Digital Injection sent.");
        delete funcc;
    }
}



void MainWindow::on_SetPixelConfig_clicked()
{
    std::string diginj = ui->DigInjText->toPlainText().toStdString();
    std::string anainj = ui->AnaInjText->toPlainText().toStdString();
    std::string Ld = ui->LdText->toPlainText().toStdString();
    std::string HBEn = ui->HBEnText->toPlainText().toStdString();

    int row = 1;
    int column = 1;
    for(unsigned int i=0; i<anainj.size(); i++)
    {
        switch((int)anainj[i])
        {
            case 48: H35pixel.SetAnaInj(row,column++,false);
                break;
            case 49: H35pixel.SetAnaInj(row,column++,true);
                break;
            case 10: // linebreak
                row++;
                column = 1;
                break;
        }
    }
    row = 1;
    column = 1;
    for(unsigned int i=0; i<diginj.size(); i++)
    {
        switch((int)diginj[i])
        {
            case 48: H35pixel.SetDigInjEn(row,column++,false);
                break;
            case 49: H35pixel.SetDigInjEn(row,column++,true);
                break;
            case 10: // linebreak
                row++;
                column = 1;
                break;
        }
    }
    row = 1;
    column = 1;
    for(unsigned int i=0; i<Ld.size(); i++)
    {
        switch((int)Ld[i])
        {
            case 48: H35pixel.SetLd(row,column++,false);
                break;
            case 49: H35pixel.SetLd(row,column++,true);
                break;
            case 10: // linebreak
                row++;
                column = 1;
                break;
        }
    }
    row = 1;
    column = 1;
    for(unsigned int i=0; i<HBEn.size(); i++)
    {
        switch((int)HBEn[i])
        {
            case 48: H35pixel.SetHBEn(row,column++,false);
                break;
            case 49: H35pixel.SetHBEn(row,column++,true);
                break;
            case 10: // linebreak
                row++;
                column = 1;
                break;
        }
    }
    logit("The configured pattern is now: ------");
    logit("AnaInjPattern:");
    logit(H35pixel.GetPatternAnaInj());
    logit("Ld Pattern:");
    logit(H35pixel.GetPatternLd());
    logit("HBEn Pattern:");
    logit(H35pixel.GetPatternHBEn());
    logit("DigInj Pattern:");
    logit(H35pixel.GetPatternDigInj());
    //Load Pixel config
    this->on_ConfigReg_clicked();
}

// ---------------------------------------------------------------
// This is setting defaults for pixels for different readouts
void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    singlepixel activepixel;
    activepixel.AnaInj = 0;
    activepixel.DigInjEn = 0;
    activepixel.HBEn = 0;
    activepixel.Ld = 0;
    int pixel;
    pixel = ui->PixelConfigSpin->value();
    switch(index)
    {
        case 0: // Analog Injection
        //All pixels
            H35pixel.SetConfig(0,1,0,0);
            //active pixel
            activepixel.AnaInj = 1;
            H35pixel.SetConfig(pixel, activepixel);
        break;
        case 1:  // Digital Hitbus
            H35pixel.SetConfig(0,0,0,0);    // Diginj, HBEn, Ld, Anainj
            activepixel.HBEn = 1;
            activepixel.AnaInj = 1;
            H35pixel.SetConfig(pixel, activepixel);
        break;
        case 2: // Digital Injection
            H35pixel.SetConfig(0,1,0,0);
            activepixel.DigInjEn = 1;
            H35pixel.SetConfig(pixel, activepixel);
        break;
        case 3: // Digital Readout with pixel address
            H35pixel.SetConfig(0,0,0,0);
            activepixel.AnaInj = 1;
            H35pixel.SetConfig(pixel, activepixel);
        break;
        default:
            H35pixel.SetConfig(0,0,0,0);
    }
    ui->AnaInjText->setText(QString::fromStdString(H35pixel.GetPatternAnaInj()));
    ui->LdText->setText(QString::fromStdString(H35pixel.GetPatternLd()));
    ui->DigInjText->setText(QString::fromStdString(H35pixel.GetPatternDigInj()));
    ui->HBEnText->setText(QString::fromStdString(H35pixel.GetPatternHBEn()));
    this->on_ConfigReg_clicked();
}

void MainWindow::on_PixelConfigSpin_valueChanged(int arg1)
{
    this->on_comboBox_currentIndexChanged(ui->comboBox->currentIndex());
}

// -----------------------------------------------------------
// This is default settings for different readout modes
// -----------------------------------------------------------
void MainWindow::on_comboBox_2_currentIndexChanged(int index)
{
    switch(index)
    {
        case 0: // Analog Injection + Analog Out
            ui->gABEnB->setText(QString::number(0));
            ui->gCompOffB->setText(QString::number(0));
            ui->gCompOffNorm->setText(QString::number(0));
            gconf.ABEn = false;
            gconf.CompOffB = false;
            gconf.CompOffNorm = false;
            break;
        case 1: // Analog Injection + Normal Comparator
            ui->gABEnB->setText(QString::number(1));
            ui->gCompOffB->setText(QString::number(0));
            ui->gCompOffNorm->setText(QString::number(1));
            gconf.ABEn = true;
            gconf.CompOffB = false;
            gconf.CompOffNorm = true;
            break;
        case 2: // Timwalk Comparator
            ui->gABEnB->setText(QString::number(1));
            ui->gCompOffB->setText(QString::number(1));
            ui->gCompOffNorm->setText(QString::number(0));
            gconf.ABEn = true;
            gconf.CompOffB = true;
            gconf.CompOffNorm = false;
            break;
    }
    this->on_ConfigReg_clicked();
}

// ---------------------------------------------------------------
// Generate 128 Pulses and read the value back from the FPGA
// ---------------------------------------------------------------
void MainWindow::on_Hitbus_clicked()
{
    if(!this->blocked)
    {
    Funkcije* funcc = new Funkcije(genio);
    funcc->InitPatternHitbus();
    funcc->InitCounter();
    funcc->StartPattern();
    funcc->SendBuffer();
    funcc->ParkPattern(); // might be tight... smt like sleep should be here, but not sleep
    funcc->SendBuffer();


    std::stringstream counterstate;
    counterstate << funcc->ReadCounterState();
    logit("Hitbus Counter: " + counterstate.str());

    delete funcc;
    }
}

// ---------------------------------------------------------------
// Start the Pattern Generator for endless injections
// ---------------------------------------------------------------
void MainWindow::on_StartPattern_clicked()
{
    // Start Pattern Generator
    Funkcije* funcc = new Funkcije(genio);
    // 21ms for 128 pulses
    funcc->InitPatternHitbus(60000); // 65535 is  max. 16 bit int
    funcc->InitCounter();
    funcc->StartPattern();
    funcc->SendBuffer();
    ui->checkBox->setChecked(true); // Start Pattern generation as long as checkbox is active.
    logit("Injections started.");
    delete funcc;
    emit StartTimer();
}



// ------------------------------------------------------------
// Abort injections running.
// ------------------------------------------------------------
void MainWindow::on_checkBox_clicked()
{
    if(!ui->checkBox->isChecked())
    {
        logit("Injections aborted.");
    }
}




// ----------------------------------------------------------------
// Simple Settings for value changes.
// ----------------------------------------------------------------
void MainWindow::on_hslide0_valueChanged(int value)
{
    ui->bdac0->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide1_valueChanged(int value)
{
    ui->bdac1->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide2_valueChanged(int value)
{
    ui->bdac2->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide3_valueChanged(int value)
{
    ui->bdac3->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide4_valueChanged(int value)
{
    ui->bdac4->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide5_valueChanged(int value)
{
    ui->bdac5->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide6_valueChanged(int value)
{
    ui->bdac6->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide7_valueChanged(int value)
{
    ui->bdac7->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide8_valueChanged(int value)
{
    ui->bdac8->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide9_valueChanged(int value)
{
    ui->bdac9->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide10_valueChanged(int value)
{
    ui->bdac10->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide11_valueChanged(int value)
{
    ui->bdac11->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide12_valueChanged(int value)
{
    ui->bdac12->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_hslide13_valueChanged(int value)
{
    ui->bdac13->setText(QString::number(value));
    //this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac0_textChanged(const QString &arg1)
{
    //ui->hslide0->setValue(ui->bdac0->text().toInt());
    gconf.SetbDAC(0, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac1_textChanged(const QString &arg1)
{
    //ui->hslide1->setValue(ui->bdac1->text().toInt());
    gconf.SetbDAC(1, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac2_textChanged(const QString &arg1)
{
    //ui->hslide2->setValue(ui->bdac2->text().toInt());
    gconf.SetbDAC(2, arg1.toInt());
   this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac3_textChanged(const QString &arg1)
{
    //ui->hslide3->setValue(ui->bdac3->text().toInt());
    gconf.SetbDAC(3, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac4_textChanged(const QString &arg1)
{
    //ui->hslide4->setValue(ui->bdac4->text().toInt());
    gconf.SetbDAC(4, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac5_textChanged(const QString &arg1)
{
    //ui->hslide5->setValue(ui->bdac5->text().toInt());
    gconf.SetbDAC(5, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac6_textChanged(const QString &arg1)
{
    //ui->hslide6->setValue(ui->bdac6->text().toInt());
    gconf.SetbDAC(6, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac7_textChanged(const QString &arg1)
{
    //ui->hslide7->setValue(ui->bdac7->text().toInt());
    gconf.SetbDAC(7, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac8_textChanged(const QString &arg1)
{
    //ui->hslide8->setValue(ui->bdac8->text().toInt());
    gconf.SetbDAC(8, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac9_textChanged(const QString &arg1)
{
    //ui->hslide9->setValue(ui->bdac9->text().toInt());
    gconf.SetbDAC(9, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac10_textChanged(const QString &arg1)
{
    //ui->hslide10->setValue(ui->bdac10->text().toInt());
    gconf.SetbDAC(10, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac11_textChanged(const QString &arg1)
{
    //ui->hslide11->setValue(ui->bdac11->text().toInt());
    gconf.SetbDAC(11, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac12_textChanged(const QString &arg1)
{
    //ui->hslide12->setValue(ui->bdac12->text().toInt());
    gconf.SetbDAC(12, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_bdac13_textChanged(const QString &arg1)
{
    //ui->hslide13->setValue(ui->bdac13->text().toInt());
    gconf.SetbDAC(13, arg1.toInt());
    this->on_ConfigReg_clicked();
}

void MainWindow::on_PixelSpin_editingFinished()
{
}


// --------------------------------------------------------
// Just a selector for the different pixel Ld TDAC values.
// --------------------------------------------------------
void MainWindow::on_PixelSpin_valueChanged(int arg1)
{
    if(arg1 < 0 || arg1 > H35pixel.size())
        return;
    ui->gtdac1->setText(QString::number(H35pixel.GetTdac1(arg1)));
    ui->gtdac2->setText(QString::number(H35pixel.GetTdac2(arg1)));
    ui->gtdac3->setText(QString::number(H35pixel.GetTdac3(arg1)));
}

// ----------------------------------------------
// Write TDAC values for the Pixels into RAM
// ----------------------------------------------
void MainWindow::on_WriteTDAC_clicked()
{
    for(int i=0; i<H35pixel.size(); i++)
    {
        ui->gtdac1->setText(QString::number(H35pixel.GetTdac1(i)));
        ui->gtdac2->setText(QString::number(H35pixel.GetTdac2(i)));
        ui->gtdac3->setText(QString::number(H35pixel.GetTdac3(i)));
        H35pixel.SetLd(false);
        H35pixel.SetLd(i,true);
        this->on_ConfigReg_clicked();
        // "It is important to keep TDAC bits unchanged before LD goes from 1 to 0."
        H35pixel.SetLd(false);
        this->on_ConfigReg_clicked();
    }
    ui->gtdac1->setText(QString::number(H35pixel.GetTdac1(ui->PixelSpin->value())));
    ui->gtdac2->setText(QString::number(H35pixel.GetTdac2(ui->PixelSpin->value())));
    ui->gtdac3->setText(QString::number(H35pixel.GetTdac3(ui->PixelSpin->value())));
}

// ----------------------------------------------
// Write Ld RAM config for Pixels to File
// ----------------------------------------------
void MainWindow::on_WriteTDACFile_clicked()
{
    std::ofstream file;
    file.open("H35config.ini");
    for(int i=0; i<H35pixel.size(); i++)
    {
        file << H35pixel.GetTdac1(i) << " " << H35pixel.GetTdac2(i) << " " << H35pixel.GetTdac3(i) << "\n";
    }
    file.close();
}

// ----------------------------------------------
// Editing the Fields will store the information for Ld RAM values in the pixel config TEMPORARILY
// ----------------------------------------------
void MainWindow::on_gtdac1_editingFinished()
{
    H35pixel.SetTdac(ui->PixelSpin->value(), ui->gtdac1->text().toInt(), ui->gtdac2->text().toInt(), ui->gtdac3->text().toInt());
}

void MainWindow::on_gtdac2_editingFinished()
{
    H35pixel.SetTdac(ui->PixelSpin->value(), ui->gtdac1->text().toInt(), ui->gtdac2->text().toInt(), ui->gtdac3->text().toInt());

}

void MainWindow::on_gtdac3_editingFinished()
{
    H35pixel.SetTdac(ui->PixelSpin->value(), ui->gtdac1->text().toInt(), ui->gtdac2->text().toInt(), ui->gtdac3->text().toInt());

}

// --------------------------------------------------
// Read TDAC values for pixels from File
// --------------------------------------------------
void MainWindow::on_ReadTDACFile_clicked()
{
    std::stringstream logger;
    logger << "TuneDAC values loaded from file.";
    logit(logger.str());

    std::ifstream file;
    file.open("H35config.ini");
    int i = 0;
    int tdac1, tdac2, tdac3;
    while(file.good())
    {
        file >> tdac1 >> tdac2 >> tdac3;
        H35pixel.SetTdac(i, tdac1, tdac2, tdac3);
        i++;
        if(i > H35pixel.size())
            break;
    }
    file.close();
    ui->gtdac1->setText(QString::number(H35pixel.GetTdac1(ui->PixelSpin->value())));
    ui->gtdac2->setText(QString::number(H35pixel.GetTdac2(ui->PixelSpin->value())));
    ui->gtdac3->setText(QString::number(H35pixel.GetTdac3(ui->PixelSpin->value())));
}

// --------------------------------------------------------
// Take S-Curve
// --------------------------------------------------------
void MainWindow::on_SCurveButton_clicked()
{
    if(!this->blocked)
    {
        std::stringstream logger;
        logger << "Recording S-Curves for all Pixels.";
        logit(logger.str());
        this->SetupGraph("Injection Voltage (V)", "Efficiency", 0, ui->SCurveVoltage->value(), 0, 1);
        emit StartSCurveAll(ui->SCurveVoltage->value(), pcbconfig.GetTh1(), pcbconfig.GetTh2());
        this->blocked = true;
    }
}

void MainWindow::on_TunePixel_clicked()
{
    if(!this->blocked)
    {
    Funkcije* funcc = new Funkcije(genio);
    int pixelnumber = ui->TunePixSpin->value();
    H35pixel.SetAnaInj(false);
    H35pixel.SetHBEn(false);
    H35pixel.SetDigInjEn(false);
    H35pixel.SetLd(false);
    H35pixel.SetLd(pixelnumber,true);
    H35pixel.SetAnaInj(pixelnumber,true);
    H35pixel.SetHBEn(pixelnumber,true);
    ui->gtdac3->setText(QString::number(0));
    this->on_ConfigReg_clicked();
    H35pixel.SetLd(false);
    this->on_ConfigReg_clicked();

    xydata SCurve = funcc->SCurve(1, pcbconfig.GetTh1(), pcbconfig.GetTh2());
    xydata lastSCurve;
    xydata lastlastSCurve;
    double tunevoltage = 1;
    for(int i=0; i<SCurve.size(); i++)
    {
        if(SCurve.GetY(i) < 0.5)
        {
            tunevoltage = SCurve.GetX(i);
            break;
        }
    }

    std::ofstream file;
    std::stringstream filename;
    filename << "untuned_S_" << pixelnumber << ".txt";
    file.open(filename.str().c_str());
    for(int i=0; i<SCurve.size(); i++)
        file << SCurve.GetX(i) << " " << SCurve.GetY(i) << "\n";
    file.close();
    bool tuned = false;
    int dac = 0;
    while(!tuned && dac < 16)
    {
        dac++;
        // Tune 1 by 1.
        ui->gtdac3->setText(QString::number(dac));
        H35pixel.SetLd(false);
        H35pixel.SetLd(pixelnumber,true);
        this->on_ConfigReg_clicked();
        // "It is important to keep TDAC bits unchanged before LD goes from 1 to 0."
        H35pixel.SetLd(false);
        this->on_ConfigReg_clicked();

        if(dac >= 2)
            lastlastSCurve = lastSCurve;
        if(dac >= 1)
            lastSCurve = SCurve;

        //Perform S-Curve measurement again
        SCurve = funcc->SCurve(1.);


        // Get Tuned Voltage
        for(int i=0; i<SCurve.size(); i++)
        {
            if(SCurve.GetY(i) < 0.5)
            {
                tunevoltage = SCurve.GetX(i);
                break;
            }
            if(SCurve.GetY(i) > 1.1)
            {
                dac -= 2;
                if(dac < 0)
                {
                    dac = 0;
                    SCurve = lastSCurve;
                }
                SCurve = lastlastSCurve;
                ui->gtdac3->setText(QString::number(dac));
                H35pixel.SetLd(false);
                H35pixel.SetLd(pixelnumber,true);
                this->on_ConfigReg_clicked();
                H35pixel.SetLd(false);
                this->on_ConfigReg_clicked();


                tuned = true;
            }
        }
        // Check if tuning is enough.
        if(tunevoltage < 0.1)
        {
            tuned = true;
        }


    }
    if(tuned)
    {
        H35pixel.SetTdac(pixelnumber, 0, 0, dac);
        ui->gtdac3->setText(QString::number(dac));
        std::cout << "TDAC3: " << dac << std::endl;
        std::stringstream filename2;
        filename2 << "tuned_S_" << pixelnumber << ".txt";
        file.open(filename2.str().c_str());
        for(int i=0; i<SCurve.size(); i++)
            file << SCurve.GetX(i) << " " << SCurve.GetY(i) << "\n";
        file.close();

    }
    }

}

void MainWindow::on_pushButton_4_clicked()
{
    if(!this->blocked)
    {
        for(int i=0; i<H35pixel.size(); i++)
        {
            ui->TunePixSpin->setValue(i);
            this->on_TunePixel_clicked();
        }
    }
}

void MainWindow::on_SCurvePixel_clicked()
{
    if(!this->blocked)
    {
    // Config for AnaInj and Hitbus readout
    H35pixel.SetAnaInj(false);
    H35pixel.SetHBEn(false);
    H35pixel.SetDigInjEn(false);
    H35pixel.SetLd(false);

    // Select Pixel for S-Curve
    int pixels = ui->TunePixSpin->value();
    H35pixel.SetAnaInj(pixels,true);
    H35pixel.SetHBEn(pixels,true);
    this->on_ConfigReg_clicked();

    //Delete old Scurve from memory
    this->scurve.Clear();
    //Log
    std::stringstream logger;
    logger << "S-Curve for Pixel " << pixels << " started";
    logit(logger.str());

    // setup plot environment
    this->SetupGraph("Injection Voltage (V)", "Efficiency", 0, ui->SCurveVoltage->value(), 0, 1);
    // Send a signal to worker thread to start.
    this->blocked = true;
    emit StartSCurve(pixels, ui->SCurveVoltage->value(), pcbconfig.GetTh1(), pcbconfig.GetTh2());
    }
}

// -------------------------------------------------------
// Handle Results from thread
// -------------------------------------------------------
void MainWindow::HandleResults(double x, double y)
{
    this->scurve.SetXY(x,y);
}

void MainWindow::WriteSCurve()
{
    int pixels = ui->TunePixSpin->value();
    std::stringstream logger;
    logger << "S-Curve Scan Pixel " << pixels << " last value: ";
    std::stringstream filename;
    filename << "SCurve" << pixels << ".txt";
    std::ofstream file;
    file.open(filename.str().c_str());
    for(int i=0; i<this->scurve.size(); i++)
    {
        file << this->scurve.GetX(i) << " " << this->scurve.GetY(i) << "\n";
    }
    file.close();
    logger << this->scurve.GetX(this->scurve.size()-1);
    logger << ". File: " << filename.str();
    logit(logger.str());
    this->scurve.Clear();
}

void MainWindow::UpdateSCurveProgress(int value)
{
    ui->SCurveProgress->setValue((value+1)/(double)H35pixel.size()*100); // as percentage
    ui->TunePixSpin->setValue(value);
}

// -------------------------------------------------------
// Global Configuration
void MainWindow::on_gABEnB_editingFinished()
{
    gconf.ABEn = ui->gABEnB->text().toInt();
}

void MainWindow::on_gCompOffNorm_editingFinished()
{
    gconf.CompOffNorm = ui->gCompOffNorm->text().toInt();
}

void MainWindow::on_gCompOffB_editingFinished()
{
    gconf.CompOffB = ui->gCompOffB->text().toInt();
}

void MainWindow::on_gEnLowPass_editingFinished()
{
    gconf.EnLowPass = ui->gEnLowPass->text().toInt();
}

void MainWindow::on_DACSpare0_clicked()
{
    gconf.SetSpare(0, ui->DACSpare0->isChecked());
}

void MainWindow::on_DACSpare1_clicked()
{
    gconf.SetSpare(1, ui->DACSpare1->isChecked());
}

void MainWindow::on_DACSpare2_clicked()
{
    gconf.SetSpare(2, ui->DACSpare2->isChecked());
}

void MainWindow::on_pcbdac_edit_editingFinished()
{
    pcbconfig.SetInj(ui->pcbdac_edit->text().toDouble());
}

void MainWindow::on_pcbdac_edit_2_editingFinished()
{
    pcbconfig.SetTh1(ui->pcbdac_edit_2->text().toDouble());
}

void MainWindow::on_pcbdac_edit_3_editingFinished()
{
    pcbconfig.SetTh2(ui->pcbdac_edit_3->text().toDouble());
}



void MainWindow::on_Stop_clicked()
{
    emit StopWork();
}

void MainWindow::on_DelayCount_clicked()
{
    if(!this->blocked)
    {
    Funkcije* funcc = new Funkcije(this->genio);
    // Initialize Delay Measurement
    funcc->InitPatternDelayCount();
    funcc->ResetPattern();
    funcc->ResetDelayCounter();

    funcc->StartDelayCounter();
    funcc->StartPattern();


    funcc->SendBuffer();

    // Read back delay count
    int delay = 0;
    delay = funcc->ReadDelayCount(0);
    int negdelay;
    negdelay = funcc->ReadDelayCount(1);

    int ndelay = 0;
    ndelay = funcc->ReadDelayCount(2);
    int nnegdelay = 0;
    nnegdelay = funcc->ReadDelayCount(3);

    std::stringstream logger;
    logger << "TW Delay: " << delay << " pos, " << negdelay << " neg.\n";
    logger << "CNor Delay: " << ndelay << " pos, " << nnegdelay << "neg.";
    logit(logger.str());

    delete funcc;
    }
}

void MainWindow::on_TuneTimewalk_clicked()
{
    if(!this->blocked)
    {
        std::string xlabel = "Injection (V)";
        std::string ylabel = "Delay (*20ns)";
        this->SetupGraph(xlabel, ylabel, 0, 4, 0, 30);
        this->blocked = true;
        emit ScanTWDown(pcbconfig.GetTh1(), pcbconfig.GetTh2());
    }
}

void MainWindow::on_TuneTh2_clicked()
{
    if(!this->blocked)
    {
        logit("Th2 Scan started.");
        this->SetupGraph("Th2 (V)", " Delta t (*20ns)", 0, 2, -5, 5);
        for(int i=0; i<1; i++)
        {
            this->blocked = true;
            emit ScanTh2(i,pcbconfig.GetTh1());
        }
    }

}

void MainWindow::on_Th1Sweep_clicked()
{
    if(!this->blocked)
    {
        Ke6485 com;
        this->blocked = true;
        logit("IV Scan with Th1 started.");
        // Setup Graph
        this->SetupGraph("Th1 (V)", "Current on Drain (A)", 0, 3.3, -5e-7, ui->MaxCurrentMOS->value()*1e-6);
        this->NewPlotCurve("Drain");

        this->on_KEread_clicked(); // initialize Keithley before measurement
        emit Th1Readings((int)(3.3/this->ui->Th1StepSize->value())); // Set the number of measurements for worker
        Funkcije* funcc = new Funkcije(genio);
        if(ui->Th1SweepDown->isChecked())
            pcbconfig.SetTh1(3.3 - ui->Th1StepSize->value());
        else
            pcbconfig.SetTh1(ui->Th1StepSize->value());
        funcc->Set3DACs(pcbconfig.GetTh1(), pcbconfig.GetTh2(),pcbconfig.GetInj());
        funcc->LoadDACPCB();
        delete funcc;

        if(kecom->isOpen())
            kecom->write(com.read());
        else
        {
            logit("No Current Measurement Device ready.");
            this->blocked = false;
        }
    }
}

// Get Results from Th1Sweep

void MainWindow::CurrentReading(double y)
{
    double step = this->ui->Th1StepSize->value();
    if(ui->Th1SweepDown->isChecked())
        step *= -1;

        // Send Plot data to Graph
        this->AddPlotData(pcbconfig.GetTh1(), fabs(y));
        // Set Th1
        // is in Range of Th1 0V - 3.3V
        // does not exceed maximum current and is not stopped
        if(pcbconfig.GetTh1() < 3.3 && fabs(y) < (this->ui->MaxCurrentMOS->value()*1e-6) && pcbconfig.GetTh1() > 0. && !this->StopMeas) // Check whether voltage and current are in range
        {
            pcbconfig.SetTh1(pcbconfig.GetTh1() + step);

            //Set DAC for Th1
            Funkcije* funcc = new Funkcije(genio);
            funcc->Set3DACs(pcbconfig.GetTh1(), pcbconfig.GetTh2(),pcbconfig.GetInj());
            funcc->LoadDACPCB();
            funcc->SendBuffer();
            delete funcc;
            // trigger a write, wait for reply
            Ke6485 com;
            kecom->write(com.read());
        }
        else
        {
            this->ready();
            emit Th1Readings(0);
            this->StopMeas = false;
            this->SaveIVData();
        }

}

void MainWindow::SaveIVData()
{
    std::ofstream file;
    std::string filename;
    switch(ui->Th1MeasOption->currentIndex())
    {
        case 0:
            filename = "DrainNCirc.txt";
            break;
        case 1:
            filename = "DrainNLin.txt";
            break;
        case 2:
            filename = "DrainP.txt";
            break;
        default:
            filename = "DrainMeas.txt";
    }
    file.open(filename.c_str());
    for(int i=0; i<plotvec_x.size(); i++)
    {
        file << plotvec_x[i] << " " << plotvec_y[i] << "\n";
    }
    file.close();
}

// --------------------------------------------------------------
// QPlot
// --------------------------------------------------------------

void MainWindow::SetupGraph(std::string xlabel, std::string ylabel, double xlow, double xhigh, double ylow, double yhigh)
{
    ui->qplotwidget->clearGraphs();
    ui->qplotwidget->xAxis->setLabel(QString::fromStdString(xlabel));
    ui->qplotwidget->yAxis->setLabel(QString::fromStdString(ylabel));
    // set axes ranges, so we see all data:
    ui->qplotwidget->xAxis->setRange(xlow, xhigh);
    ui->qplotwidget->yAxis->setRange(ylow, yhigh);
    ui->qplotwidget->legend->setVisible(true);

}

void MainWindow::AddPlotData(double x, double y)
{
    if(ui->qplotwidget->graphCount() == 0)
        ui->qplotwidget->addGraph();
    plotvec_x.push_back(x);
    plotvec_y.push_back(y);
    ui->qplotwidget->graph(ui->qplotwidget->graphCount()-1)->setData(plotvec_x, plotvec_y);
    ui->qplotwidget->replot();
}

void MainWindow::RefreshPlot()
{
    ui->qplotwidget->graph(ui->qplotwidget->graphCount()-1)->setData(plotvec_x, plotvec_y);
    ui->qplotwidget->replot();
}

void MainWindow::NewPlotCurve(QString graphname)
{
    plotvec_x.clear();
    plotvec_y.clear();
    ui->qplotwidget->addGraph();
    ui->qplotwidget->graph(ui->qplotwidget->graphCount()-1)->setName(graphname);
    switch(ui->qplotwidget->graphCount()-1)
    {
        case 1: ui->qplotwidget->graph(0)->setPen(QPen(Qt::blue));
        break;
        case 2: ui->qplotwidget->graph(1)->setPen(QPen(Qt::red));
        break;
        case 3: ui->qplotwidget->graph(2)->setPen(QPen(Qt::green));
        break;
        case 4: ui->qplotwidget->graph(3)->setPen(QPen(Qt::darkYellow));
        break;
        case 5: ui->qplotwidget->graph(4)->setPen(QPen(Qt::darkCyan));
        break;
        default: ui->qplotwidget->graph(ui->qplotwidget->graphCount()-1)->setPen(QPen(Qt::black));
    }
}

// ----------------------------------------------------------------
// COM Communication with Keithley over RS232
// ----------------------------------------------------------------

QByteArray MainWindow::StringToByteArray(std::string command)
{
    QByteArray data;
    for(unsigned int i=0; i<command.size(); i++)
    {
        data.push_back(command.at(i));
    }
    data.push_back('\n');
    return data;
}

void MainWindow::on_KEread_clicked()
{
    Ke6485 com;
    //std::cout << kecom->isOpen() << std::endl;
    if(kecom->isOpen())
    {
        kecom->write(com.reset());
        kecom->write(com.arm());
        kecom->write(com.armcount(1));
        kecom->write(com.trigger());
        kecom->write(com.triggercount(ui->KeReadings->value()));
        kecom->write(com.form());
        kecom->write(com.zerocheck(false));
        kecom->write(com.zerocorrect(false));
    }
    emit SetReadings(ui->KeReadings->value());
}

void MainWindow::readCOMData()
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
    reading.resize(ui->KeReadings->value());
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
    }

}

void MainWindow::on_KEread2_clicked()
{
    Ke6485 com;
    kecom->write(com.read());
}



void MainWindow::on_KeReadings_valueChanged(int arg1)
{
    emit SetReadings(arg1);
    Ke6485 com;
    kecom->write(com.triggercount(arg1));
}



void MainWindow::COMdataReady(double y)
{
    std::stringstream ss;
    ss << y;
    logit("Meas: "  + ss.str());
}


void MainWindow::on_pushButton_clicked()
{
    this->StopMeas = true;
}

void MainWindow::on_Th1MeasOption_currentIndexChanged(int index)
{
    switch(index)
    {
        case 0:
            ui->Th1SweepDown->setChecked(false);
            break;
        case 1:
            ui->Th1SweepDown->setChecked(false);
            break;
        case 2:
            ui->Th1SweepDown->setChecked(true);
            break;
        default:
            ui->Th1SweepDown->setChecked(false);
    }
}

//-----------------------------------------------------------------
// Fast Digital Readout.
//-----------------------------------------------------------------
void MainWindow::on_GetPixelAddress_clicked()
{
    /*
    Funkcije* funcc = new Funkcije(genio);
    std::stringstream logger;
    logger << "Pixeladdress: " << (std::bitset<16>)funcc->GetPixelAddress();
    logit(logger.str());

    delete funcc;
    */
    logit("Pixel Address printint in cout...");
    emit GetPixelAddress();
}

void MainWindow::on_FastClockDiv_editingFinished()
{

}


void MainWindow::on_AddressDelay_editingFinished()
{

}

void MainWindow::on_FastClockEnable_stateChanged(int arg1)
{
    // enable Clock for digital readout
    if(arg1 == 2)
        emit EnableDigitalClock(true);
    else
        emit EnableDigitalClock(false);
}

void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    Funkcije* funcc = new Funkcije(genio);
    std::cout << arg1 << std::endl;
    if(arg1 == 2)
        funcc->SetDigitalInjection(true);
    else
        funcc->SetDigitalInjection(false);
    delete funcc;
}

void MainWindow::on_FindlowestTh1_clicked()
{
    // Implement function from Felix Ehrler here.
    int pixel = ui->TunePixSpin->value();
    emit FindLowestTh1(pixel, gconf.GetSpare(1));
    std::stringstream logger;
    logger << "Ramping to lowest threshold for pixel " << pixel;
    logit(logger.str());
}


// ----------------------------------------------------------------------
// SELECT port for communication with Keithley device. Virtual COM Port.
// Review here.
// ----------------------------------------------------------------------

void MainWindow::on_comboBox_3_currentIndexChanged(const QString &arg1)
{
    std::cout << arg1.toStdString() << std::endl;
    if(kecom != NULL)
        delete kecom;
    kecom = new QextSerialPort(arg1.toStdString().c_str(), QextSerialPort::EventDriven);
    kecom->setBaudRate(BAUD9600);
    kecom->setDataBits(DATA_8);
    kecom->setStopBits(STOP_1);
    kecom->setFlowControl(FLOW_XONXOFF);
    kecom->setParity(PAR_NONE);
    bool keopen = kecom->open(QIODevice::ReadWrite);
    if(keopen)
    {
        logit("COM port successfully opened for Keithley 6485");
        this->on_KEread_clicked();
    }
    else
    {
        logit("Failed to open COM port. Please choose another port or make sure, the device is connected!");
    }
    //std::cout << "Opened 6485newfunction? " << kecom->open(QIODevice::ReadWrite) << std::endl;
}



void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if(arg1 == 0)
        emit StopTimer();
    else
        emit StartTimer();
}
void MainWindow::on_Pixeladressgen_stateChanged(int arg1)
{
    if(arg1 == 0)
        emit StartTimerReadPixel();
    else
        emit StopTimerReadPixel();
}


void MainWindow::on_AddressDelay_valueChanged(int arg1)
{
    emit SetDigPixDelay(ui->AddressDelay->value());

}

void MainWindow::on_FastClockDiv_valueChanged(int arg1)
{
     emit SetDigPixClockDiv(ui->FastClockDiv->value());
}

void MainWindow::on_AutoDelay_clicked()
{
    double TH1;
    int delay = 0;
    double avgdelay = 0.0;
    int TWdownmax = 0;
    Funkcije* funcc = new Funkcije(this->genio);
    // Initialize Delay Measurement
    TH1 = pcbconfig.GetTh1();
    funcc->InitPatternDelayCount();

    //File Output initalize
    std::ofstream fileout;
    std::ostringstream foutname;
    foutname << "Pix_" << ui->PixelConfigSpin->value()<< "-TH1_" << TH1 << ".txt";
    fileout.open(foutname.str().c_str());

    std::cout << "Pixel: " << ui->PixelConfigSpin->value() << " TH1: " << TH1 << "\n";
    fileout << "Pixel: " << ui->PixelConfigSpin->value() << " TH1: " << TH1 << "\n";
    std::cout << "TW TWdown Injection Delay TH2\n";
    fileout << "TW TWdown Injection Delay TH2\n";

    std::ofstream avgfileout;
    std::ostringstream avgfoutname;
    avgfoutname << "AVG_Pix_" << ui->PixelConfigSpin->value()<< "-TH1_" << TH1 << ".txt";
    avgfileout.open(avgfoutname.str().c_str());

    std::cout << "Pixel: " << ui->PixelConfigSpin->value() << " TH1: " << TH1 << "\n";
    avgfileout << "Pixel: " << ui->PixelConfigSpin->value() << " TH1: " << TH1 << "\n";
    std::cout << "TW TWdown Injection AVGDelay TH2\n";
    avgfileout << "TW TWdown Injection AVGDelay TH2\n";

    for(int TW = 1; TW <=63; TW += 2) // TW Sweep
    {
        gconf.SetbDAC(1, TW);
        TWdownmax = (TW * 3) + 5;
        if(TWdownmax > 63)
            TWdownmax = 63;
        for(int TWdown = TW; TWdown <= TWdownmax; TWdown += 2)  // TWdown Sweep
        {
            gconf.SetbDAC(3, TWdown);
            this->on_ConfigReg_clicked();

            for (double injection = 0.2; injection <= 3.2; injection += 0.2)    //Injection Sweep
            {
                for (double th2 = 0.1; th2 <= 3.3; th2 += 0.1)        //TH2 Sweep
                {
                    funcc->Set3DACs(pcbconfig.GetTh1(), th2, injection );
                    funcc->LoadDACPCB();
                    avgdelay = 0.0;
                    for (int i = 1; i <= 10; i++)
                    {

                        funcc->ResetPattern();
                        funcc->ResetDelayCounter();
                        funcc->StartDelayCounter();
                        funcc->StartPattern();
                        funcc->SendBuffer();
                        // Read back delay count

                        delay = funcc->ReadDelayCount(0);
                        avgdelay = avgdelay + delay;
                        /*int negdelay;
                        negdelay = funcc->ReadDelayCount(1);

                        int ndelay = 0;
                        ndelay = funcc->ReadDelayCount(2);
                        int nnegdelay = 0;
                        nnegdelay = funcc->ReadDelayCount(3);*/

                        std::cout << TW << " " << TWdown <<  " " << injection <<  " " << delay << " " << th2 << "\n";

                        fileout << TW << " " << TWdown <<  " " << injection <<  " " << delay << " " << th2 << "\n";

                    }
                    avgdelay = avgdelay / 10;
                    std::cout << TW << " " << TWdown <<  " " << injection <<  " " << avgdelay << " " << th2 << "\n";
                    avgfileout << TW << " " << TWdown <<  " " << injection <<  " " << avgdelay << " " << th2 << "\n";


                }
            }
        }
    }
    fileout.close();
    avgfileout.close();
    delete funcc;
    this->ready();
}
