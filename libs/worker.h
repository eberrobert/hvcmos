#ifndef WORKER_H
#define WORKER_H

#include "func.h"
#include "plot.h"
#include "geniobase.h"
#include "config.h"
#include "KeCOM.h"
#include <qextserialport.h>

#include <vector>

#include <QObject>
#include <QTimer>


class Worker : public QObject, public Funkcije, public Ke6485
{
    Q_OBJECT
    public:
        Worker();
        Worker(GenioBase* genio, QextSerialPort* kecom, globalconfig gconf, pixelconfig H35pixel, PCBconfig* pcbconf);
        ~Worker();
    public slots:

        /**
         * @brief FindLowestTh1 Ramps up or down Th1 in order to find the lowest possible threshold not in noise.
         * @param pixel is the pixel where to perform the action.
         */
        void FindLowestTh1(int pixel, bool flagSpare1);

        /**
         * @brief SetDigPixClockdiv Sets the clock divider for the fast readout clock. The slow clock is 8 times slower.
         * At 1, the clock is divided by 2 (Maybe fix in the FPGA machine).
         * @param div is the divider factor.
         */
        void SetDigPixClockdiv(int div);

        /**
         * @brief SetDigPixDelay Sets the delay between start of the FPGA state machine, where the pixel address is recorded.
         * @param delay is the delay in time units of the FPGA fast counter. The usual fast clock is 48*4 MHz = 192MHz.
         */
        void SetDigPixDelay(int delay);

        /**
         * @brief EnableDigitalClock Enables the clocks on the FPGA to read out the Pixel addresses.
         * @param enable enables the clocks, if true.
         */
        void EnableDigitalClock(bool enable);

        /**
         * @brief StartTimerReadPixel Starts reading the pixel addresses continuously with the timer. 1sec.
         */
        void StartTimerReadPixel();

        /**
         * @brief StopTimerReadPixel Stops reading the pixel addresses continuously with the timer.
         */
        void StopTimerReadPixel();

        /**
         * @brief StartTimer Starts the continuous injections to the Chip. The pattern is not changed. Configure before use.
         */
        void StartTimer();

        /**
         * @brief StopTimer Stops the continuous injections.
         */
        void StopTimer();

        /**
         * @brief GetPixelAddress Reads the Pixel Address from the two Busses of the HVStripV1 Chip.
         */
        void GetPixelAddress();

        /**
         * @brief SetTh1Readings Sets the number of steps while sweeping through Th1.
         * @param steps is the number of steps, before the measurement ends.
         */
        void SetTh1Readings(int steps);

        /**
         * @brief SetReadings Sets the number of readings from the Keithley current measurement device.
         * @param readings is the number of readings.
         */
        void SetReadings(int readings);

        /**
         * @brief readCOMData Reads data from the COM port, if available. Waits for the correct reading.
         */
        void readCOMData();

        /**
         * @brief ScanTh2 Scans the delay for 2 different injections varying th2.
         * @param pixel the pixel number, the action is performed on.
         * @param th1 Th1, threshold 1.
         */
        void ScanTh2(int pixel, double th1);

        /**
         * @brief ScanTWDown scans the delay for different settings of TWDown DAC at different injection voltages.
         */
        void ScanTWDown(double th1, double th2);

        /**
         * @brief ConfigChip configures the H35V1 Chip according to the information given in gconf, H35pixel and pcbconf
         * This is required to configure the chip in an extra thread, the Worker class is providing.
         */
        void ConfigChip();

        /**
         * @brief SetSCurveConfig configures the chip to measure S-Curves.
         * @param pixel is the pixel, the S-Curves is measured on.
         */
        void SetSCurveConfig(int pixel);

        /**
         * @brief StopWork should stop the current process. Not working atm.
         */
        void StopWork();

        /**
         * @brief DoAllSCurves starts S-Curve scan for all pixels.
         * @param startvoltage
         * @param th1
         * @param th2
         */
        void DoAllSCurves(double startvoltage, double th1, double th2);

        /**
         * @brief DoSCurve1 Starts the S-Curve scan on a single pixel.
         * @param startvoltage
         * @param th1
         * @param th2
         */
        void DoSCurve1(int pixel, double startvoltage, double th1, double th2);

        /**
         * @brief quit closes this thread.
         */
        void quit();
    signals:
        /**
         * @brief Logit sends a QString to the logit instance in the MainWindow.
         * @param logger is the QString to send.
         */
        void Logit(QString logger);

        void ResultX(double x);
        void ResultY(double y);

        void CurrentReading(double y);


        /**
         * @brief COMdataReady signals a general ready, that COM data has arrived.
         * @param y is a double with the result from the COM reading.
         */
        void COMdataReady(double y);

        /**
         * @brief NewPlotCurve adds a new curve to the plotter.
         */
        void NewPlotCurve(QString graphname);

        /**
         * @brief AddPlotData signals a new point for the data to plot in the MainWindow
         * @param x x-point
         * @param y y-point
         */
        void AddPlotData(double x, double y);

        /**
         * @brief resultready signals the result from a S-Curve.
         * Because the result cannot be transferred as xydata, it is handled a x and y pairs.
         * @param x x-value
         * @param y y-value
         */
        void resultready(double x, double y);

        /**
         * @brief SCurvefinished signals, that the S-Curve on one pixel is finished.
         */
        void SCurvefinished();

        /**
         * @brief UpdateProgress signals an updated status to the main thread.
         * @param value
         */
        void UpdateProgress(int value);

        /**
         * @brief finished is signaled, when the thread is closed.
         */
        void finished();


        /**
         * @brief ready is signaled, when the current action is finished.
         */
        void ready();

    private slots:
        void on_timer_update();

    private:
        void readCOMData(int readings);
        pixelconfig H35pixel;
        globalconfig gconf;
        PCBconfig* pcbconf;
        GenioBase* genio;
        QextSerialPort* kecom;
        bool stop;
        bool running;
        int readings;
        bool wait;
        int Th1Readings;
        //Timer related variables
        bool timerRunning;
        bool TimerReadPixel;
        //QTimer *timer;
};

#endif // WORKER_H
