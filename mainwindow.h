#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <sstream>

//My
#include "libs/ftdi.h"
#include "libs/geniobase.h"
#include "libs/func.h"
#include "libs/config.h"

// QCustomPlot
#include "libs/qcustomplot.h"

//QSerialExtPort
#include "qextserialport-1.2rc/src/qextserialport.h"

#if defined(__linux__)
//#include <TQtWidget.h>
//#include <TMultiGraph.h>
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_actionExit_triggered();

    void on_actionHV_CMOS_GUI_triggered();

    void on_ConfigReg_clicked();

    void on_PCB_DAC_clicked();

    void on_pushButton_3_clicked();

    void on_SetPixelConfig_clicked();

    void on_comboBox_currentIndexChanged(int index);

    void on_comboBox_2_currentIndexChanged(int index);

    void on_Hitbus_clicked();

    void on_StartPattern_clicked();



    void on_checkBox_clicked();

    void on_hslide0_valueChanged(int value);

    void on_hslide1_valueChanged(int value);

    void on_hslide2_valueChanged(int value);

    void on_hslide3_valueChanged(int value);

    void on_hslide4_valueChanged(int value);

    void on_hslide5_valueChanged(int value);

    void on_hslide6_valueChanged(int value);

    void on_hslide7_valueChanged(int value);

    void on_hslide8_valueChanged(int value);

    void on_hslide9_valueChanged(int value);

    void on_hslide10_valueChanged(int value);

    void on_hslide11_valueChanged(int value);

    void on_hslide12_valueChanged(int value);

    void on_hslide13_valueChanged(int value);

    void on_bdac0_textChanged(const QString &arg1);

    void on_bdac1_textChanged(const QString &arg1);

    void on_bdac2_textChanged(const QString &arg1);

    void on_bdac3_textChanged(const QString &arg1);

    void on_bdac4_textChanged(const QString &arg1);

    void on_bdac5_textChanged(const QString &arg1);

    void on_bdac6_textChanged(const QString &arg1);

    void on_bdac7_textChanged(const QString &arg1);

    void on_bdac8_textChanged(const QString &arg1);

    void on_bdac9_textChanged(const QString &arg1);

    void on_bdac10_textChanged(const QString &arg1);

    void on_bdac11_textChanged(const QString &arg1);

    void on_bdac12_textChanged(const QString &arg1);

    void on_bdac13_textChanged(const QString &arg1);

    void on_PixelSpin_editingFinished();

    void on_PixelSpin_valueChanged(int arg1);

    void on_WriteTDAC_clicked();

    void on_WriteTDACFile_clicked();

    void on_gtdac1_editingFinished();

    void on_gtdac2_editingFinished();

    void on_gtdac3_editingFinished();

    void on_ReadTDACFile_clicked();

    void on_SCurveButton_clicked();

    void on_TunePixel_clicked();

    void on_pushButton_4_clicked();

    void on_gABEnB_editingFinished();

    void on_gCompOffNorm_editingFinished();

    void on_gCompOffB_editingFinished();

    void on_gEnLowPass_editingFinished();

    void on_DACSpare0_clicked();

    void on_DACSpare1_clicked();

    void on_DACSpare2_clicked();

    void on_pcbdac_edit_editingFinished();

    void on_pcbdac_edit_2_editingFinished();

    void on_pcbdac_edit_3_editingFinished();

    void on_SCurvePixel_clicked();

    void on_Stop_clicked();

    void on_DelayCount_clicked();

    void on_TuneTimewalk_clicked();

    void on_PixelConfigSpin_valueChanged(int arg1);

    void SetupGraph(std::string xlabel, std::string ylabel, double xlow, double xhigh, double ylow, double yhigh);

    void on_TuneTh2_clicked();

    void on_KEread_clicked();

    void readCOMData();

    void on_KEread2_clicked();

    void on_Th1Sweep_clicked();

    void on_KeReadings_valueChanged(int arg1);

    void on_pushButton_clicked();

    void on_Th1MeasOption_currentIndexChanged(int index);

    void SaveIVData();

    void on_GetPixelAddress_clicked();

    void on_checkBox_2_stateChanged(int arg1);

    void on_FindlowestTh1_clicked();

    void on_FastClockEnable_stateChanged(int arg1);

    void on_comboBox_3_currentIndexChanged(const QString &arg1);

    void on_FastClockDiv_editingFinished();

    void on_AddressDelay_editingFinished();

    void on_checkBox_stateChanged(int arg1);

    void on_Pixeladressgen_stateChanged(int arg1);


    void on_AddressDelay_valueChanged(int arg1);

    void on_FastClockDiv_valueChanged(int arg1);

    void on_AutoDelay_clicked();

public slots:
    void HandleResults(double x, double y);
    void UpdateSCurveProgress(int value);
    void WriteSCurve();
    void AddPlotData(double x, double y);
    void NewPlotCurve(QString graphname);
    void ready();
    //void ResultX(double x);
    //void ResultY(double y);
    void COMdataReady(double y);
    void CurrentReading(double y);
    void Logit(QString logger);

signals:
    void StartSCurve(int pixel, double startvoltage, double th1, double th2);
    void StartSCurveAll(double startvoltage,double th1 ,double th2);
    void StopWork();
    void ScanTWDown(double th1, double th2);
    void ScanTh2(int pixel, double th1);
    void Th1Sweep();
    void SetReadings(int readings);
    void Th1Readings(int steps);
    void GetPixelAddress();
    void StartTimerReadPixel();
    void StopTimerReadPixel();
    void StartTimer();
    void StopTimer();
    void EnableDigitalClock(bool enable);
    void SetDigPixClockDiv(int div);
    void SetDigPixDelay(int delay);
    void FindLowestTh1(int pixel, bool spare);


private:
    Ui::MainWindow *ui;

    // Useful functions
    QByteArray StringToByteArray(std::string command);
    void RefreshPlot();

    //by Robert Eber
    FTDI* ftdi;
    GenioBase* genio;

    //Log
    void logit(std::string logstream);

    //constants

    // Holds the pixel configuration.
    pixelconfig H35pixel;

    // Global Config.
    globalconfig gconf;

    //PCB Configuration DACs
    PCBconfig pcbconfig;

    // Result Handling
    xydata scurve;
    QVector<double> plotvec_x;
    QVector<double> plotvec_y;
    //QCustomPlot* customPlot;
    int plotcounter;

    //Thread handling
    bool blocked;
    bool StopMeas;

    //COM PORT
    QextSerialPort *kecom;

};

#endif // MAINWINDOW_H
