//---------------------------------------------------------------------------

#ifndef FunkcijeH
#define FunkcijeH
//---------------------------------------------------------------------------
#include <stdint.h>
//#include "WinTypes.h"
//#include <Chart.hpp>
//#include "ftusbdev.h"
#include "ftdi.h"
//#include "MainHVPix.h"
//#include "Silib_GPIB_Interfaces.h"
#include <time.h>
//#include "Silib_GPIB_TDS.h"

//#include "Silib_GPIB_HP_33120A.h"
//#include "Silib_GPIB_HP_E3631A.h"
//#include "Silib_GPIB_Keithley_2400.h"
//#include "Silib_GPIB_Agilent_53132A.h"
//#include "Silib_GPIB_HP_33220A.h"
//#include "Silib_GPIB_HP_E3631A.h"
#include "math.h"
//#include <math.h>
#include "geniobase.h"
#include <time.h>
#include "plot.h"

const unsigned char PCB_CK = 0x80;   //reg addr 64+3
const unsigned char PCB_DI = 0x40;     //reg addr 64+3
const unsigned char PCB_LD = 0x20;     //reg addr 64+3

static const int bufferLength  = 4096; //genio buffer
static const int bufferMaxFill = 4000;

const unsigned char PAT_CK1    = 0x40; //reg addr 64+0
const unsigned char PAT_CK2    = 0x80; //reg addr 64+0
const unsigned char PAT_DI  = 0x20; //reg addr 64+0

const unsigned char PAT_CKRo1  = 0x80; //reg addr 64+1
const unsigned char PAT_CKRo2  = 0x40; //reg addr 64+1
const unsigned char PAT_LdConf  = 0x20; //reg addr 64+1
const unsigned char PAT_LdRec  = 0x10; //reg addr 64+1

const unsigned char PAT_Trg  = 0x08; //reg addr 64+1

const unsigned char PAT_Reset  = 0x04; //reg addr 64+1

const unsigned char PAT_SIn  = 0x02; //reg addr 64+1

const unsigned char PAT_ParEn  = 0x01; //reg addr 64+1
const unsigned char PAT_ShEnB = 0x00; // reg addr 64+1

const unsigned char PAT_Load  = 0x08; //reg addr 64+1

const int INT_SENS = 100;
const double PI   = 3.141;
const int ID_OSZI   =  1;


/**
 * This class provides some higher-level functions to test the HV-CMOS chip.
 *
 * 
 */
class Funkcije
{
 private: // added by Robert Eber
  int EditMon; // default value from FormMain->EditMon in MainHVPix.cpp
  bool LdRO; // default value = ?
  bool HB; // default = ?
  bool EnChip; // default = ?
  char Bit;
  bool Box9; // Box 9
  bool InvertInj; 
  int Edit1; // delay
  int Edit2;
  int EditColl;
  int EditRow;

  //FTDI* ftdi; // This is the pointer to the FTDI device class. Initialized with SetExternalFTDI.
  GenioBase* genio; // Pointer to the GenioBase class. initialized with SetExternalGenio or InitGenio.

 public:
  void sleep(int milliseconds); // try to get sleep right
  /**
   * Basic constructor.*/
  Funkcije(); 
  
  /**
   * Basic destructor. */
  ~Funkcije(); 

  /**
   * @brief Funkcije Constructor initiliazing class with external genio class.
   * Requires a pointer to GenioBase.
   * @param genio
   */
  Funkcije(GenioBase* genio);
  
  /**
   * genwrap_WriteData was originally a nasty global wraparound function to communicate with the FTDI class.
   * Now it is implemented in this class. However, to use it, the private pointers to FTDI and GenioBase have to be set.
   * 
   * Writes data to the FTDI chip.
   *
   * Requires a pointer to the buffer to write (Buf), the buffer length to write into the FTDI chip.
   *
   * Returns the number of bytes written (BytesWritten).
   */
 //bool genwrap_WriteData(unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesWritten);
  
  /**
   * genwrap_ReadData was originally a nasty global wraparound function to communicate with the FTDI class. Implemented in this class now. Requires the private pointers to FTDI and GenioBase to be initialized.
   *
   * Requires a pointer to the buffer to be read into (Buf), the length of the buffer to be read (BufLen).
   *
   * Returns the numbers of bytes returned from the FTDI (BytesRead).
   */
  //bool genwrap_ReadData( unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesRead);
 
  // These functions are provided by Genio and should be used only in Genio class.
  //bool openFTDIFIFO();
  //bool openFTDI();
  //void closeFTDI();
  
  //External ftdi class handle
  /**
   * SetExternalFTDI requires a pointer to the open FTDI connection. The FTDI class is used to communicate in this class.
   *
   * Requires a pointer to the FTDI class.
   */
  //void SetExternalFTDI(FTDI* ext);
  
  /**
   * If the GenioBase class is already initiallized with an FTDI connection, there is no need to do so again. This class is capable of dealing with the same object. 
   *
   * SetExternalGenio requires a pointer to the external GenioBase class.
   */
  void SetExternalGenio(GenioBase* extgen);
  
  /**
   * InitGenio initializes the GenioBase class class, which is used in the Funkcije class. Therefore, an external ftdi is required, since the connection should be made in the FTDI class.
   *
   * Requires the externally initialized FTDI pointer.
   */
  void InitGenio(FTDI* extftdi);

  // Set private variables
  /**
   * SetEditColl sets the private variable EditColl.
   * 
   * Requires an integer since it sets the pixel column.
   */
  void SetEditColl(int coll);
  
  /**
 * SetEditRow sets the private variable EditRow.
 *
 * Requires an integer since it sets the pixel row.
 */
  void SetEditRow(int row);
  
  /**
   * @brief SendBit
   * SendBit sends the configured bit to the FPGA into register 0x41.
   * Configuration is stored in genio->addpair().
   * Use SendBuffer() to transfer the bits.
   * Requires the bit as bool to be written.
   * @param bit
   */
  void SendBit(bool bit);

  void SendBitSensor(bool bit);
  void SetSlow(int nrofrows);

  /**
   * @brief SetPattern Sets the private char Bit.
   * Requires a char.
   * @param Pattern
   */
  void SetPattern(char Pattern);

  /**
   * @brief LoadConf
   * Triggers the load config bit.
   */
  void LoadConf();

  bool SetSlowBits(char Bit);

  unsigned short int ReadBuffer();

  /**
   * @brief SendDACPCB DendDACPCB sends the configuration to the DACs on the PCB (HV-Board).
   * Requires an integer value for DACValue.
   * @param DACValue
   */
  void SendDACPCB(int DACValue);

  /**
   * @brief LoadDACPCB loads the configured values into the DACs all at once.
   */
  void LoadDACPCB();

  /**
   * @brief SendBitPCB clocks in the single bits from SendDACPCB.
   * Requires a single bool.
   * @param bit
   */
  void SendBitPCB(bool bit);

  /**
   * SendBuffer sends the buffer with genio->sendbuf().
   */
  bool SendBuffer();

  /**
   * @brief SendDAC : Sends the configuration for a DAC to the chip. Easier to use than single bools.
   * @param DACValue : 6-bit integer value for the configuration of the DAC.
   * @param SpareBit : SpareBit can be used by the chip. Usually it is 0.
   */
  bool SendDAC(int DACValue, bool SpareBit );

  void SendDACSensor(int DACValue, bool SpareBit );
  void SendVerticalConfig(bool LdThr, bool Bit1, bool LdTimer );
  void SendInDAC(int DACValue, bool SpareBit );
  void Refresh();

  /**
   * @brief SendLoadConf sets the load line to high for loading the previously sent configuration into the chip.
   * Bit 0x20 = high at Address 64+1
   */
  void SendLoadConf();

  /**
   * @brief InitPattern intializes the pattern sequence generation in the FPGA. 8 (0..7) pattern sequence generators are available.
   * @param patgenid: The id of the pattern sequence generator, from 0 to 7.
   * @param tt0
   * @param tt1
   * @param tt2
   * @param tt3
   * @param tt4
   * @param tt5
   * @param tt6
   * @param tt7
   * @param period: 8bit. Period length. Defines the length of a period in units of Tgen.
   * @param runlen: 16bit. Defines the total length of the sequence. Periods = runlen/clkdiv
   * @param clkdiv: 16bit. Defines the generator period length as Tgen = 20ns * clkdiv.
   * @param initdelay: 16bit. Delay, after which the pattern sequence starts to run.
   * @param initstate: bool. Defines, if the sequence starts high (1) or low (0).
   * @param rststate: bool. Defines the reset state of the sequence. Resets to high (1) or low (0).
   */
  void InitPatternGen(
		      unsigned char patgenid,
		      unsigned char tt0,
		      unsigned char tt1,
		      unsigned char tt2,
		      unsigned char tt3,
		      unsigned char tt4,
		      unsigned char tt5,
		      unsigned char tt6,
		      unsigned char tt7,
		      unsigned char period,
		      unsigned short runlen,
              unsigned short clkdiv,
		      unsigned short initdelay,
		      bool initstate,
		      bool rststate
		      );

  /**
   * @brief InitPatternHitbus intializes the Hitbus and injects a number of pulses, 128 by default.
   * The pattern generators 0 and 2 in the FPGA are used. A maximum of 65535 pulses are possible (16 bit integer).
   */
  void InitPatternHitbus(int pulses = 128);

  /**
   * @brief InitCounter intializes and resets the internal counter in the FPGA.
   */
  void InitCounter();

  /**
   * @brief StartPattern starts the pattern sequence generation. This actually triggers the injection into the Chip.
   */
  void StartPattern();

  /**
   * @brief ParkPattern sets the pattern generators to reset to 0.
   * Otherwise, the pattern generators are reset to 1 or 0, depending on their configuration in InitPatternHitbus.
   */
  void ParkPattern();

  void ResetPattern();
  void StopPattern();

  /**
   * @brief ReadCounterState reads the counter from the FPGA register 0x64 and 0x65 and converts them into an integer.
   * @return returns the counter as an integer. 16 bit integer.
   */
  int ReadCounterState();

  /**
   * @brief ReadDelayCount reads the delay between injection high and CONA[x] (readback) high from register 0x39.
   * @param counternumber reads the delay from 1 of the 4 counters. 0: CONA[18] posedge. 1: CONA[18] negedge. 2: CONA[17] posedge. 3: CONA[17] negedge.
   * @return returns the delay as multiples of 20ns as integer.
   */
  int ReadDelayCount(int counternumber);

  /**
   * @brief ResetDelayCounter resets the delay counter. Sending to register 0x3B;
   */
  void ResetDelayCounter();

  /**
   * @brief StartDelayCounter starts the delay counter.
   */
  void StartDelayCounter();

  /**
   * @brief InitPatternDelayCount initializes a single shot pattern for measurement of the delay between injection
   * and hitbus high. For the tuning of the timewalk compensated comparator.
   */
  void InitPatternDelayCount();

  /**
   * @brief Efficiency is a simple measurement of the efficiency for whatever pixel is just connected.
   * @param Measurements is the number of measurements
   * @param Injections is the number of injections per measurement
   * @return returns the efficiency.
   */
  double Efficiency(int Measurements, int Injections = 128);

  /**
   * @brief SCurve triggers the ThresholdScan function with scantype = 1. The starting voltage needs to be given.
   * The SCurve is then recorded by decreasing the startvalue down to a value, where 3 consecutive 0s are recoreded.
   * @param startvalue
   */  
  xydata SCurve(double startvalue, double th1 = 2, double th2 = 2);

  /**
   * @brief ThresholdScan performs a measurement of the S-Curve.
   * @param scantype sets the type of the scan. 1 or 2. 1 is the default value.
   * @param startvalue is the starting voltage. The scan will decrease the voltage downwards.
   * @param Measurements is the number of measurements at each voltage. Each measurement consists of 128 injections.
   */
  xydata ThresholdScan( int scantype, double startvalue, int Measurements, double th1 = 2, double th2 = 2 );

  /**
   * @brief Set3DACs sets the input of 3 DACs on the PCB in the right order: Threshold 1, threshold 2, injection amplitude.
   * @param th1 Threshold 1 voltage.
   * @param th2 Threshold 2 voltage.
   * @param inj Injection Voltage amplitude.
   */
  void Set3DACs(double th1, double th2, double inj);

  /**
   * @brief SetAmplitude Calculates and sets the setting for DAC on the PCB.
   * @param Injection is the voltage amplitude of the DAC to be set.
   */
  void SetAmplitude(double Injection);

  /**
   * @brief MeasureDelay measures the delay between pattern for injection and hitbus high.
   * Measurement precision is about 10ns because we count at posedge and negedge.
   * @return returns the delay as double.
   */
  double MeasureDelay(int measurements = 1);

  /**
   * @brief GetPixelAddress reads the pixel address from OUT1
   * @return returns an integer with 16bits of the pixel address, 8 are the pixel address.
   */
  unsigned int GetPixelAddress();

  /**
   * @brief SetDigpixClockdiv sets the clock divider for the fast digital pixel address readout.
   * @param clockdiv divides the board clock by a factor of clockdiv * 4.
   */
  void SetDigpixClockdiv(int clockdiv);

  /**
   * @brief ResetDigitalClock resets the digital clock generator in the FPGA.
   */
  void ResetDigitalClock();

  /**
   * @brief SetDigpixDelay sets the delay for the start of the pixel address readout.
   * @param delay is the delay in fast clock ticks. This is 4 times the board clock (Uxibo: 4*48 = 192MHz).
   */
  void SetDigpixDelay(int delay);

  /**
   * @brief SetDigitalInjection enables digital injection into the pixel.
   * @param digital is a bool value. True for digital injection enabled.
   */
  void SetDigitalInjection(bool digital);

  /**
   * @brief EnableDigitalClock enables the digital readout clocks for pixel address generation.
   * @param enable is the enable bit. True if clocks are running.
   */
  void EnableDigitalClock(bool enable);


  unsigned short int ReadMatrixCounterState();
  double ToTScan(char Bit, double *meansigma);
  void InitPatternToT(void);
  bool ResetFIFO(char Bit);
  DWORD ReadToTCounterState(unsigned short int *CntState);
  DWORD ReadFlag();
  void MeasureSpectrum(char Bit, /*TObject *Sender,*/ double alpha, double beta, double gamma);




  long double D_ChiSquare(int index, double mu, double sig, signed int sign);    //checked
  long double D2_ChiSquare(int index1, int index2, double mu, double sig, signed int sign);   //checked
  long double D_ChiSquareAna(int index, double mu, double sig, signed int sign);       //checked
  long double D2_ChiSquareAna(int index1, int index2, double mu, double sig, signed int sign);  //checked
  void FitLM(signed int sign); //checked
  double DGaussDmu(double x, double mu, double sig); //checked
  double DGaussDsig(double x, double mu, double sig); //checked
  void GetAllResults(signed int sign);
  double Gauss(double x, double mu, double sig);
  long double ChiSquare(double mu, double sig, signed int sign); //chi square
  int FindClosest(double* array, double value, int length);
  

  void SendConfigReg( unsigned char *Array, int Row, int *DacState, bool LdThr, bool LdTimer );
  void SendRoClock(bool Load, bool NextRow, bool SIn);
  unsigned short int ReadPixel(unsigned char Address);
  unsigned char Readout(int Coll, int Row);
  

  void InitCounterDelayC(unsigned char InpBit, unsigned char InpBit2); //initiates injection - L1 delay!
  bool SetStrobeSlow(char Bit);
  void StartPatternL1(void);  //starts pattern for hitbus
  void InitPatternL1(unsigned char delay, unsigned char width);  //pattern for hitbus  128 pulses
  int DoManyRO(int Coll, int Row);
  double FindInput(double input, int mode);
  bool LoadFifo(char Bit);
  bool ResetFifo(char Bit);
  //void Funkcije::PlotPixel(int x, int y, int z, int norows);
  void ReadoutFIFO();  //starts pattern for hitbus
  int ReadPixelFifo();  //hb counter
  
  
  double xmeas[1000];
  double ymeas[1000];
  double yfit[1000];
  long double muvalue;
  long double sigvalue;
  double mustart;
  double sigstart;
  int nmeas;
  int iteration;


// Oszi Functions
  //double ReadOszi();
  //void SetOszi();
  //void openOszi();
  //void openGPIB();
  // void Funkcije::SetOsziSmall();
  //TGPIB_Interface *GPIB_Interface;
  //TGPIB_TDS *Oszi;
  
};
#endif
