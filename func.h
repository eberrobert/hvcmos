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

const unsigned char PAT_Load  = 0x08; //reg addr 64+1

const int INT_SENS = 100;
const double PI   = 3.141;
const int ID_OSZI   =  1;


/**
 * This class provides some higher-level functions to test the HV-CMOS chip.
 * The whole class should be rewritten!!
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

  FTDI* ftdi; // This is the pointer to the FTDI device class. Initialized with SetExternalFTDI.
  GenioBase* genio; // Pointer to the GenioBase class. initialized with SetExternalGenio or InitGenio.
  void sleep(int milliseconds); // try to get sleep right
 public:
  /**
   * Basic constructor.*/
  Funkcije(); 
  
  /**
   * Basic destructor. */
  ~Funkcije(); 
  
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
  
  bool genwrap_WriteData(unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesWritten);
  
  /**
   * genwrap_ReadData was originally a nasty global wraparound function to communicate with the FTDI class. Implemented in this class now. Requires the private pointers to FTDI and GenioBase to be initialized.
   *
   * Requires a pointer to the buffer to be read into (Buf), the length of the buffer to be read (BufLen).
   *
   * Returns the numbers of bytes returned from the FTDI (BytesRead).
   */
  bool genwrap_ReadData( unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesRead);
 
  // These functions are provided by Genio and should be used only in Genio class.
  bool openFTDIFIFO();
  bool openFTDI();
  void closeFTDI();
  
  //External ftdi class handle
  /**
   * SetExternalFTDI requires a pointer to the open FTDI connection. The FTDI class is used to communicate in this class.
   *
   * Requires a pointer to the FTDI class.
   */
  void SetExternalFTDI(FTDI* ext);
  
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
  
  void SendBit(bool bit);
  void SendBitSensor(bool bit);
  void SetSlow(int nrofrows);
  bool SetSlowBits(char Bit);
  unsigned short int ReadBuffer();
  void SendDACPCB(int DACValue);
  void SendBitPCB(bool bit);
  void SendBuffer();
  void SendDAC(int DACValue, bool SpareBit );
  void SendDACSensor(int DACValue, bool SpareBit );
  void SendVerticalConfig(bool LdThr, bool Bit1, bool LdTimer );
  void SendInDAC(int DACValue, bool SpareBit );
  void Refresh();
  void SendLoadConf();
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
		      unsigned short clkfac,
		      unsigned short initdelay,
		      bool initstate,
		      bool rststate
		      );
  void InitPatternHitbus();  //checked
  void InitCounter(void);     //checked
  void StartPattern(void);   //checked
  void ParkPattern(void);  //checked
  DWORD ReadCounterState();  //checked
  unsigned short int ReadMatrixCounterState();
  double ToTScan(char Bit, double *meansigma);
  void InitPatternToT(void);
  bool ResetFIFO(char Bit);
  DWORD ReadToTCounterState(unsigned short int *CntState);
  DWORD ReadFlag();
  void MeasureSpectrum(char Bit, /*TObject *Sender,*/ double alpha, double beta, double gamma);
  void SetAmplitude(double Injection);
  void ThresholdScan( int scantype, double startvalue, double fixedvalue );
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
