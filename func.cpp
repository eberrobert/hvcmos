//---------------------------------------------------------------------------

#include "func.h"
#include "geniobase.h"
//#include "MainHVPix.h"
#include "math.h"
#include <iostream>
//#include "../FTDI/WinTypes.h"
#include <boost/thread/thread.hpp>
#include <time.h>
//#include <chrono>

extern bool MeasurementStopped;

//GenioBase *genio; // from geniobase.h

//---------------------------------------------------------------------------

// No Log Window for Linux app.
/*
void genwrap_LogMessage( std::string *message ) 
{
  FormMain->MemoMartin->Lines->Add( message->c_str() );
}
*/
bool Funkcije::genwrap_WriteData( unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesWritten) 
{
  return ftdi->Write( (unsigned char*)Buf, BufLen, BytesWritten );
}
bool Funkcije::genwrap_ReadData( unsigned char *Buf, long unsigned int BufLen, long unsigned int *BytesRead) 
{
  return ftdi->Read( (unsigned char*)Buf, BufLen, BytesRead );
}

//---------------------------------------------------------------------------
 Funkcije::Funkcije() // Constructor
{
  //genio = new GenioBase();
  //genio->extfunc_writedata = genwrap_WriteData;
  //genio->extfunc_readdata  = genwrap_ReadData;
  //genio->extfunc_logmessage = genwrap_LogMessage;
  //genio->BufferLog = true;
  
  // initialising default values for replacements of EDITS or BOXES
  EditMon = 100; // default value from FormMain->EditMon in MainHVPix.cpp
  LdRO = false; // default value = ?
  HB = false; // default = ?
  EnChip = false; // default = ?
  Bit = 0;
  Box9 = false; // Box 9
  InvertInj = false; 
  Edit1 = 0; // delay
  Edit2 = 0;
  EditColl = 10;
  EditRow = 10;
}

Funkcije::~Funkcije(void) // Destructor
{
  //delete genio;
}

bool Funkcije::openFTDI()
{
  ftdi = new FTDI();
  bool opened = ftdi->Open(1);
  ftdi->resetDevice();
  UCHAR Mask = 0xFF;
  UCHAR Mode = 0x01; // set Asynchronous Bit Bang mode
  DWORD Baud = 0x809C; //baud rate 19200
  //Error when using UCHAR for Mask,Mode,Baud.
  if(! ftdi->setBitMode(Mask,Mode))
    return false;
  if(! ftdi->setBaudRate(Baud))
    return false;
  genio->initializeFtdi(ftdi);
  return opened;
}

bool  Funkcije::openFTDIFIFO() // same as open but with less features??
{
 ftdi = new FTDI();
 bool opened =  ftdi->Open(1);
 ftdi->resetDevice();
 genio->initializeFtdi(ftdi);
 return opened;
}

void Funkcije::closeFTDI()
{
  ftdi->Close();
  delete ftdi;
}

void Funkcije::SetExternalFTDI(FTDI* ext)
{
  this->ftdi = ext;
}

void Funkcije::SetExternalGenio(GenioBase* extgen)
{
    this->genio = extgen;
}


void Funkcije::InitGenio(FTDI* extftdi)
{
  this->genio = new GenioBase(extftdi);
}

void Funkcije::SetEditColl(int coll)
{
  this->EditColl = coll;
}

void Funkcije::SetEditRow(int row)
{
  this->EditRow = row;
}

void Funkcije::sleep(int milliseconds)
{
  timespec tim, tim2;
  tim.tv_sec = 0;
  tim.tv_nsec = milliseconds * 1e6;
  nanosleep(&tim, &tim2); // sleep
}

void  Funkcije::SendBit(bool bit)
{
   unsigned char pattern;  //bit patterns for 8 bit registers in xilinx
   //pattern=0x00;
   //pattern = FormMain -> Bit;
   pattern = this->Bit;
   pattern = pattern | PAT_ParEn;
   if (bit)
     {
       pattern = pattern | PAT_SIn;
     }
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern | PAT_CKRo1;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo1;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern | PAT_CKRo2;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo2;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   //genio->sendbuf();

}


void  Funkcije::SendLoadConf()
{
   unsigned char pattern;  //bit patterns for 8 bit registers in xilinx
   //pattern=0x00;
   //pattern = FormMain -> Bit;
   pattern = this->Bit;
   pattern = pattern | PAT_LdConf;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   //pattern = FormMain -> Bit;
   pattern = this->Bit;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   //genio->sendbuf();

}

void  Funkcije::Refresh()
{
  //unsigned long bytesWritten;
  //unsigned char Address;
  //Address = 0x41;
  genio->addpair( 0x41, 0xFF );
  genio->addpair( 0x41, 0xFF );
  genio->addpair( 0x41, 0x00 );
  genio->addpair( 0x41, 0x00 );
  genio->addpair( 0x41, 0xFF );
  genio->addpair( 0x41, 0xFF );
  genio->addpair( 0x41, 0x00 );
  genio->addpair( 0x41, 0x00 );
  genio->sendbuf();
}

void  Funkcije::SendBitSensor(bool bit)
{
  unsigned char pattern;  //bit patterns for 8 bit registers in xilinx
  pattern=0x00;
  if (bit)
    {
      pattern = pattern | PAT_DI;
    }
  genio->addpair( 0x40, pattern);
  pattern = pattern | PAT_CK1;
  genio->addpair( 0x40, pattern);
  pattern = pattern & ~PAT_CK1;
  genio->addpair( 0x40, pattern);
  pattern = pattern | PAT_CK2;
  genio->addpair( 0x40, pattern);
  pattern = pattern & ~PAT_CK2;
  genio->addpair( 0x40, pattern);
}

void  Funkcije::SendDACSensor(int DACValue, bool SpareBit )
{
  SendBitSensor(SpareBit);
  for (int ibit=0; ibit<6; ibit++) 
    {
      SendBitSensor( (DACValue & (0x01<<(ibit))) != 0x00  );
    }
  //genio->addpair( 0x40, 0);    //adds zero to DI nicht wichtig
}

void  Funkcije::SendDAC(int DACValue, bool SpareBit )
{
  SendBit(SpareBit);
  for (int ibit=0; ibit<7; ibit++) 
    {
      SendBit( (DACValue & (0x01<<(6-ibit))) != 0x00  );
    }
  //genio->addpair( 0x40, 0);    //adds zero to DI nicht wichtig
}

void  Funkcije::SendInDAC(int DACValue, bool SpareBit )
{
  SendBit(SpareBit);
  for (int ibit=0; ibit<6; ibit++) 
    {
      if(ibit == 0) SendBit( ( DACValue & (0x01<<5) ) == 0x00  );//LSB not used
      if(ibit == 1) SendBit( ( DACValue & (0x01<<4) ) != 0x00  );//LSB not used
      if(ibit == 2) SendBit( ( DACValue & (0x01<<3) ) == 0x00  );//LSB not used
      if(ibit == 3) SendBit( ( DACValue & (0x01<<2) ) != 0x00  );//LSB not used
      if(ibit == 4) SendBit( ( DACValue & (0x01<<1) ) == 0x00  );//LSB not used
      if(ibit == 5) SendBit( ( DACValue & (0x01<<0) ) == 0x00  );//LSB not used
    }
  //genio->addpair( 0x40, 0);    //adds zero to DI nicht wichtig
}

void  Funkcije::SendVerticalConfig(bool LdThr, bool Bit1, bool LdTimer )
{
  SendBit(1);
  SendBit(1);
  SendBit(1);
  SendBit(1);
  SendBit(LdTimer);
  SendBit(Bit1);
  SendBit(LdThr);
  
  //genio->addpair( 0x40, 0);    //adds zero to DI nicht wichtig
}



void  Funkcije::SendConfigReg( unsigned char *Array, int Row, int *DacState, bool LdThr, bool LdTimer )
{
  for (int i = 0; i<15; i++)
    {
      //      if (FormMain -> EditMon -> Text.ToInt() == (14-i)) // requires EditMon. Initialized with 100 (MainHVPix.cpp)
	if (EditMon == (14-i))
	  SendDAC(DacState[14-i], true );
	else 
	  SendDAC(DacState[14-i], false );
    }
 //SendBuffer();
  this->sleep(100); //Sleep(100);

 for (int i = 0; i<30; i++) 
   {
     if (i>=0){//(i == 13) || (i == 14) || (i == 15)
       //if (FormMain -> CheckBoxLdRO -> Checked)
       if(LdRO == true)
	 {
	   SendInDAC(Array[(29-i) + 30*Row], true);
	 }
       else 
	 SendInDAC(Array[(29-i) + 30*Row], false); //false end
      }
     else 
       SendInDAC(Array[(29-i) + 30*Row], false);  //false end
   }
 //SendBuffer();
 
 //SendVerticalConfig(bool LdThr, bool Bit1, bool LdTimer )

 for (int i = 0; i<30; i++) 
   {
     if (i == 0)
       {
	 //if (FormMain -> CheckBoxHB -> Checked) 
	 if(HB)
	   SendVerticalConfig( ((29-i) == Row) && LdThr, true, ((29-i) == Row) && LdTimer);
	 else 
	   SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
       }
      if (i == 12)
	{
	  //if (FormMain -> CheckBoxEnChip -> Checked) 
	  if(EnChip)
	    SendVerticalConfig(((29-i) == Row) && LdThr, true, ((29-i) == Row) && LdTimer);
	  else 
	    SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
	}
      if (i == 13)
	{
	  //	  if (FormMain -> CheckBoxEnChip -> Checked) 
	  if(EnChip)
	    SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
	  else 
	    SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
	}
      if (i == 14)
	{
	  //	  if (FormMain -> CheckBoxEnChip -> Checked) 
	  if(EnChip)
	    SendVerticalConfig(((29-i) == Row) && LdThr, true, false);
	  else 
	    SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
	}
      if ((i != 0)&&(i != 12)&&(i != 13)&&(i != 14)) 
	SendVerticalConfig(((29-i) == Row) && LdThr, false, ((29-i) == Row) && LdTimer);
   }
 SendLoadConf();
 SendBuffer();
}
/*
void  Funkcije::SendRoClock(bool Load, bool NextRow, bool SIn)
{
   unsigned char pattern;
   pattern = FormMain -> Bit;
   //pattern = 0x00;
   if (NextRow) pattern = pattern | PAT_ParEn;
   if (SIn) pattern = pattern | PAT_SIn;
   genio->addpair( 0x41, pattern);
   pattern = pattern | PAT_CKRo1;
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo1;
   genio->addpair( 0x41, pattern);

   if (Load){
      pattern = pattern | PAT_Load;
      genio->addpair( 0x41, pattern);
   }

   pattern = pattern | PAT_CKRo2;
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo2;
   genio->addpair( 0x41, pattern);
}
*/


void  Funkcije::SendRoClock(bool Load, bool NextRow, bool SIn)
{
	unsigned char pattern;  //bit patterns for 8 bit registers in xilinx
	//pattern = FormMain->Bit;
	pattern = this->Bit;
	if (NextRow)
	  {
	    pattern = pattern | PAT_ParEn;
	  }
	genio->addpair( 0x41, pattern);
	if (SIn)
	  {
	    pattern = pattern | PAT_SIn;
	  }
	genio->addpair( 0x41, pattern);
	pattern = pattern | PAT_CKRo1;
	genio->addpair( 0x41, pattern);
	pattern = pattern & ~PAT_CKRo1;
	genio->addpair( 0x41, pattern);
	//pattern = 0x00;
	//genio->addpair( 0x41, pattern);
	if (Load)
	  {
	    pattern = pattern | PAT_Load;
	    genio->addpair( 0x41, pattern);
	  }

	pattern = pattern | PAT_CKRo2;
	genio->addpair( 0x41, pattern);
	pattern = pattern & ~PAT_CKRo2;
	genio->addpair( 0x41, pattern);

	pattern = pattern & ~PAT_Load;
	genio->addpair( 0x41, pattern);

}


unsigned short int  Funkcije::ReadPixel(unsigned char Address)  //hb counter
{
  DWORD Pixel = 0;
  unsigned char buffer[2];
  unsigned long bytesWritten;
  unsigned long bytesRead;
  unsigned char Bit;
  Bit = 0x01;                         //cnt0
  buffer[0]=Address;
  buffer[1]=Bit;
  ftdi->Write(buffer, 2, &bytesWritten);
  ftdi->Read(&Pixel, 1, &bytesRead);
  return Pixel;
}

//SendRoClock(bool Load, bool NextRow, bool SIn)

int  Funkcije::ReadPixelFifo()  //hb counter
{
  
  int blocksize = 120; //32X32
  genio->addpair( 0x08+7, 0xFF );  //hitactive should be activated
  genio->addpair( 0x08+6, blocksize%256 );  //ostatak
  genio->addpair( 0x08+5, blocksize/256 );  //deljenje
  
  genio->sendbuf();
  this->sleep(100); // sleep
  genio->readbuf(120);
  
  int numberofpixels;
  numberofpixels = 0;
  for(int j=0; j<30; j++)
    {
      for(int i=0; i<32; i++)
	{
	  //PlotPixel(i, j, 0, 30);
	}
    }

  for(int j=0; j<30; j++)
    {
      for(int i=0; i<8; i++)
	{
	  if ( ( genio->RXBuf[j*4+0] & (0x01 << i) ) == 0 ) 
	    { 
	      //PlotPixel(i, j, 255, 30);  
	      if(i >= 2)
		{
		  numberofpixels ++;
		} 
	    };
            //else PlotPixel(i, j, 0, 30);
	  if ( ( genio->RXBuf[j*4+1] & (0x01 << i) ) == 0 ) 
	    { 
	      //PlotPixel(i+8, j, 255, 30);  
	      numberofpixels ++; 
	    };
            //else PlotPixel(i+8, j, 0, 30);
            if ( ( genio->RXBuf[j*4+2] & (0x01 << i) ) == 0 ) 
	      { 
		//PlotPixel(i+16, j, 255, 30);  
		numberofpixels ++; 
	      };
            //else PlotPixel(i+16, j, 0, 30);
            if ( ( genio->RXBuf[j*4+3] & (0x01 << i) ) == 0 ) 
	      { 
		//PlotPixel(i+24, j, 255, 30);  
		if(i <= 3){numberofpixels ++;
		} 
	      };
            //else PlotPixel(i+24, j, 0, 30);
	}
    }
  //FormMain -> Image1-> Repaint();  // No image in linux
  return numberofpixels;
}


void  Funkcije::SendBitPCB(bool bit)
{
   //sends a bit in pcb dac
  unsigned char pattern1;
  pattern1=0;
  if (bit)
    {
      pattern1 = pattern1 | PCB_DI;
    }
  genio->addpair( 0x43, pattern1);
  pattern1 = pattern1 | PCB_CK;
  genio->addpair( 0x43, pattern1);
  pattern1 = pattern1 & ~PCB_CK;
  genio->addpair( 0x43, pattern1);
}

void  Funkcije::SendBuffer()
{
  genio->sendbuf();
}

void  Funkcije::SendDACPCB(int DACValue)
{
  //sets one PCB dac
  for (int ibit=0; ibit<14; ibit++) 
    {
      SendBitPCB( (DACValue & (0x01<<(13-ibit))) != 0x00 );   //msb first!
    }
  SendBitPCB( 0 );   //dummy1
  SendBitPCB( 0 );   //dummy2
  genio->addpair( 0x43, 0);    //adds zero to DI nicht wichtig
  genio->addpair( 0x43, 0);
  genio->addpair( 0x43, PCB_LD);  //ld
  genio->addpair( 0x43, PCB_LD);  //ld
  genio->addpair( 0x43, PCB_LD);  //ld
  genio->addpair( 0x43, 0);       //ld off
  //genio->sendbuf();
}

void  Funkcije::SetSlow(int nrofrows)
{
  for (int i=0; i<(128-nrofrows); i++)
    {
      SendBit(false);
    }
  for (int i=0; i<(nrofrows); i++)
    {
      SendBit(true);
    }
  for (int i=0; i<(nrofrows); i++)
    {
      SendBit(false);
    }
  genio->addpair( 0x40, 0); //nicht wichtig
  genio->sendbuf();                //sends Martin's buffer to xilinx
}

bool  Funkcije::SetSlowBits(char Bit)
{
  //unsigned long bytesWritten;
  //unsigned char Address;
  //Address = 0x41;
  genio->addpair( 0x41, Bit );
  //if (FormMain->CheckBox9->Checked)
  if(Box9)
    genio->addpair( 0x44, 0xFF );
  else 
    genio->addpair( 0x44, 0 );
  genio->sendbuf();
  //InitCounterDelayC(FormMain->Edit1->Text.ToInt(),FormMain->Edit2->Text.ToInt());
  InitCounterDelayC(Edit1,Edit2);
  return true;
}

bool  Funkcije::SetStrobeSlow(char Bit)
{
  //unsigned long bytesWritten;
  unsigned char Address;
  Address = 0x48;
  genio->addpair( Address, Bit );
  return true;
}


bool  Funkcije::LoadFifo(char Bit)
{
  //unsigned long bytesWritten;
  unsigned char Address;
  Address = 0x49;
  genio->addpair( Address, Bit );
  return true;
}

bool  Funkcije::ResetFifo(char Bit)
{
  //unsigned long bytesWritten;
  unsigned char Address;
  Address = 0x4A;
  genio->addpair( Address, Bit );
  return true;
}



unsigned short int  Funkcije::ReadBuffer()
{

  int blocksize=128*64;
  //original values
  //genio->addpair( 0x08+6, blocksize%256 );  //ostatak
  //genio->addpair( 0x08+5, blocksize/256 );  //deljenje
  genio->addpair(0x53,0);
  //genio->addpair(0x38,0x08);
  genio->sendbuf();
  genio->readbuf(blocksize);
  for (int i=0; i<blocksize; i++)
    {
      //FormMain -> MemoIvan -> Lines -> Add(genio->RXBuf[i]);
      std::cout << genio->RXBuf[i] << " "  << std::flush;
    }
  std::cout << std::endl;
  return 0;
}


void  Funkcije::InitPatternGen(   //extreemely slow
        BYTE patgenid,
        BYTE tt0,
        BYTE tt1,
        BYTE tt2,
        BYTE tt3,
        BYTE tt4,
        BYTE tt5,
        BYTE tt6,
        BYTE tt7,
        BYTE period,
        WORD runlen,
        WORD clkfac,
        WORD initdelay,
        bool initstate,
        bool rststate
				  )
{
  //unsigned long BYTEsWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x80+0x10*patgenid;
  Bit = tt0;
  genio->addpair( Address, Bit );
  Address++;        //1
  Bit = tt1;
  genio->addpair( Address, Bit );
  Address++;        //2
  Bit = tt2;
  genio->addpair( Address, Bit );
  Address++;        //3
  Bit = tt3;
  genio->addpair( Address, Bit );
  Address++;        //4
  Bit = tt4;       //5
  genio->addpair( Address, Bit );
  Address++;        //5
  Bit = tt5;
  genio->addpair( Address, Bit );
  Address++;        //6
  Bit = tt6;
  genio->addpair( Address, Bit );
  Address++;        //7
  Bit = tt7;
  genio->addpair( Address, Bit );
  Address++;        //8 period
  Bit = period;
  genio->addpair( Address, Bit );
  Address++;        //9 control registers
  Bit = (rststate?1:0) << 0 | (initstate?1:0) << 1;
  genio->addpair( Address, Bit );
  Address++;        //10 high len
  Bit = runlen >> 8;
  genio->addpair( Address, Bit );
  Address++;        //11 low len
  Bit = runlen & 0xFF;
  genio->addpair( Address, Bit );
  Address++;        //12 init delay high
  Bit = initdelay >> 8;
  genio->addpair( Address, Bit );
  Address++;        //13 init delay low
  Bit = initdelay & 0xFF;
  genio->addpair( Address, Bit );
  Address++;        //14 clkfac delay high
  Bit = clkfac >> 8;
  genio->addpair( Address, Bit );
  Address++;        //15 clkfac delay low
  Bit = clkfac & 0xFF;
  genio->addpair( Address, Bit );
  genio->sendbuf();
}


void  Funkcije::InitPatternHitbus()  //pattern for hitbus  128 pulses
{
  InitPatternGen( /* patgen id */ 0,
                            /* toggetimes*/ 1, 3, 0, 0, 0, 0, 0, 0,
                            /* period len*/ 4,
                            /* run length*/ 512,
                            /* clkdiv    */ 2048, //was 1024
                            /* init delay*/ 0,
                            /* init/rst state*/ 0, 0 );
  InitPatternGen( /* patgen id */ 2,
                            /* toggetimes*/ 0, 2, 0, 0, 0, 0, 0, 0,
                            /* period len*/ 4,
                            /* run length*/ 512,
                            /* clkdiv    */ 2048,
                            /* init delay*/ 0,
                            /* init/rst state*/ 1, 1 );
}


void  Funkcije::InitPatternL1(unsigned char delay, unsigned char width)  //pattern for hitbus  128 pulses
{
  // if(FormMain->CheckBoxInvertInj->Checked) 
  if(InvertInj) 
   InitPatternGen( /* patgen id */ 2,
                            /* toggetimes*/ 0, 0, 0, 0, 0, 0, 0, 0,
                            /* period len*/ 255,
                            /* run length*/ 256,
                            /* clkdiv    */ 1, //was 1024
                            /* init delay*/ 0,
                            /* init/rst state*/ 1, 1 );
 else 
   InitPatternGen( /* patgen id */ 2,
                            /* toggetimes*/ 0, 0, 0, 0, 0, 0, 0, 0,
                            /* period len*/ 255,
                            /* run length*/ 256,
                            /* clkdiv    */ 1, //was 1024
                            /* init delay*/ 0,
                            /* init/rst state*/ 0, 0 );

 InitPatternGen( /* patgen id */ 3,
                            /* toggetimes*/ delay, delay+width, 0, 0, 0, 0, 0, 0,
                            /* period len*/ 255,
                            /* run length*/ 256,
                            /* clkdiv    */ 1,
                            /* init delay*/ 0,
                            /* init/rst state*/ 0, 0 );


}

void  Funkcije::InitCounter()  //initiates hitbus counter
{
  //unsigned long bytesWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x61;
  Bit = 0x00;         //cnt enable is off
  genio->addpair( Address, Bit );
  Address = 0x60;
  Bit = 0x00;           //cnt rst  off
  genio->addpair( Address, Bit );
  Address = 0x60;
  Bit = 0x01;           //cnt rst  on
  genio->addpair( Address, Bit );
  Address = 0x60;
  Bit = 0x00;           //cnt rst  off
  genio->addpair( Address, Bit );
  Address = 0x62;
  Bit = 0x00;            //fallende 0
  genio->addpair( Address, Bit );
  genio->sendbuf();
}

void  Funkcije::StartPattern()  //starts pattern for hitbus
{
  //unsigned long bytesWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x78;
  Bit = 0xFF;
  genio->addpair( Address, Bit );
  Address = 0x78;
  Bit = 0xFF - 0x85;       //gen 0 and 2 and 7
  genio->addpair( Address, Bit );
  genio->sendbuf();
}

void  Funkcije::StartPatternL1(void)  //starts pattern for hitbus
{
  // unsigned long bytesWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x78;
  Bit = 0xFF;
  genio->addpair( Address, Bit );
  Address = 0x78;
  Bit = 0xFF - 0x0C;       //gen 2 and 3
  genio->addpair( Address, Bit );
  genio->sendbuf();
}

void  Funkcije::ParkPattern(void)  //puts all init, rst states at 0
{
  InitPatternGen( /* patgen id */ 2,
		  /* toggetimes*/ 1, 10, 0, 0, 0, 0, 0, 0,
		  /* period len*/ 16,
		  /* run length*/ 2048,
		  /* clkdiv    */ 64000,
		  /* init delay*/ 0,
		  /* init/rst state*/ 0, 0 );
  
  InitPatternGen( /* patgen id */ 0,
		  /* toggetimes*/ 4, 12, 0, 0, 0, 0, 0, 0,
		  /* period len*/ 16,
		  /* run length*/ 2048,
		  /* clkdiv    */ 64000,
		  /* init delay*/ 0,
		  /* init/rst state*/ 0, 0 );
  InitPatternGen( /* patgen id */ 3,
		  /* toggetimes*/ 4, 12, 0, 0, 0, 0, 0, 0,
		  /* period len*/ 16,
		  /* run length*/ 2048,
		  /* clkdiv    */ 64000,
		  /* init delay*/ 0,
		  /* init/rst state*/ 0, 0 );
  
  //unsigned long bytesWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x78;
  Bit = 0xFF;
  genio->addpair( Address, Bit );
  genio->sendbuf();                              //sets reset at 1
}

DWORD  Funkcije::ReadCounterState()  //hb counter
{
  DWORD CntState = 0;
  unsigned char buffer[4];
  unsigned long bytesWritten;
  unsigned long bytesRead;
  unsigned char Bit;
  Bit = 0x01;                         //cnt0
  buffer[0]=0x65;
  buffer[1]=Bit;
  buffer[2]=0x64;
  buffer[3]=Bit;
  ftdi->Write(buffer, 4, &bytesWritten);
  this->sleep(50);
  ftdi->Read(&CntState, 2, &bytesRead);
  return CntState;
}

unsigned short int  Funkcije::ReadMatrixCounterState()  //L1 DO counter

{
  unsigned short int CntState = 0;
  unsigned char buffer[4];
  unsigned long bytesWritten;
  unsigned long bytesRead;
  unsigned char Bit;
  Bit = 0x01;
  buffer[0]=0x38; //changed!!!      //note this is asynchronous couter  Trigg
  buffer[1]=Bit;
  buffer[2]=0x37;  //changed!!!
  buffer[3]=Bit;
  ftdi->Write(buffer, 4, &bytesWritten);
  this->sleep(50);
  ftdi->Read(&CntState, 2, &bytesRead);
  return CntState;
}


double  Funkcije::ToTScan(char Bit, double *meansigma)
{
  unsigned short int CntState[16];
  const int range = 512;    //ToT range
  int Results[range];
  int AllToTes[6400];
  int Sum = 0;
  int SqSum = 0;
  int times = 10;          //was 400  //number of measurements
  double mean;
  double sigma;
  //  double sqsigma;
  for (int i = 0; i < range; i++)
    {
      Results[i] = 0;   //resets array
    }
  InitPatternToT();   //16 pulses enough!
  ResetFIFO(0xFF);
  ResetFIFO(0);
  
  for (int loop = 0; loop < times; loop++)
    {
      StartPattern();
      this->sleep(50); // sleep
      ReadToTCounterState(&CntState[0]);
      for (int j = 0; j < 16;  j++)
	{
	  Sum = Sum + CntState[j];
	  AllToTes[loop*16+j] = CntState[j]; //used for sigma calculation
	  for (int i = 0; i < range; i++)
	    {
	      if ((CntState[j] >= (i*4 - 0)) && (CntState[j] < (i*4 + 4)))
		{
		  Results[i] = Results[i] + 1;
		}
	    }
	}
      ResetFIFO(0xFF);
      ResetFIFO(0);
    }
  // no display
  //FormMain -> Chart1->BottomAxis->Title->Caption = "ToT";
  //FormMain -> Chart1->LeftAxis->Title->Caption = "Hits";
  /*
  for(int i = 0; i < range; i++)
 {
  FormMain -> Series1 -> AddXY(i*4, Results[i]);
  }*/
  mean = Sum/(16.0*times);
  for (int loop = 0; loop < times; loop++)
    {
      for (int j = 0; j < 16;  j++)
	{
	  SqSum =  SqSum + (AllToTes[loop*16+j] - mean)*(AllToTes[loop*16+j] - mean);
	}
    }
  meansigma[0] = mean;
  sigma = sqrt(SqSum/(16*times));  //!!!
  meansigma[1] = sigma;
  // No Memo. Added cout instead.
  std::cout << "Mean value is " << mean << std::endl;
  std::cout << "Sigma is " << sigma << std::endl;
  //FormMain -> MemoResults -> Lines -> Add("Mean value is " + FloatToStrF(mean, ffExponent, 4, 4));
  //FormMain -> MemoResults -> Lines -> Add("Sigma is " + FloatToStrF(sigma, ffExponent, 4, 4));
  return mean;
}


void  Funkcije::InitPatternToT(void)    //pattern for tot measurement only 32 pulses!
{
  InitPatternGen( /* patgen id */ 0,
		  /* toggetimes*/ 1, 3, 0, 0, 0, 0, 0, 0,
		  /* period len*/ 4,
		  /* run length*/ 128,
		  /* clkdiv    */ 1024,
		  /* init delay*/ 0,
		  /* init/rst state*/ 0, 0 );
  InitPatternGen( /* patgen id */ 2,
		  /* toggetimes*/ 0, 2, 0, 0, 0, 0, 0, 0,
		  /* period len*/ 4,
		  /* run length*/ 128,
		  /* clkdiv    */ 1024,
		  /* init delay*/ 0,
		  /* init/rst state*/ 1, 1 );
}


bool  Funkcije::ResetFIFO(char Bit)  //this is new, reset on/off for the old ToT FIFO
{
  //  unsigned long bytesWritten;
  unsigned char Address;
  Address = 0x45;
  genio->addpair( Address, Bit );
  genio->sendbuf();
  return true;
}


DWORD  Funkcije::ReadToTCounterState(unsigned short int *CntState) //kind of burst read reads the old ToT FIFO
{
  
  unsigned long bytesRead;
  unsigned long bytesWritten;
  unsigned char Bit;
  Bit = 0x01;
  unsigned char buffer[64];
  //ftdi->resetDevice();
  buffer[0] = 0x67;
  buffer[1] = Bit;
  buffer[2] = 0x66;
  buffer[3] = Bit;
  buffer[4] = 0x69;
  buffer[5] = Bit;
  buffer[6] = 0x68;
  buffer[7] = Bit;
  buffer[8] = 0x6B;
  buffer[9] = Bit;
  buffer[10] = 0x6A;
  buffer[11] = Bit;
  buffer[12] = 0x6D;
  buffer[13] = Bit;
  buffer[14] = 0x6C;
  buffer[15] = Bit;
  buffer[16] = 0x6F;
  buffer[17] = Bit;
  buffer[18] = 0x6E;
  buffer[19] = Bit;
  buffer[20] = 0x71;
  buffer[21] = Bit;
  buffer[22] = 0x70;
  buffer[23] = Bit;
  buffer[24] = 0x73;
  buffer[25] = Bit;
  buffer[26] = 0x72;
  buffer[27] = Bit;
  buffer[28] = 0x75;
  buffer[29] = Bit;
  buffer[30] = 0x74;
  buffer[31] = Bit;
  buffer[32] = 0x27; //note the addresses have changed
  buffer[33] = Bit;
  buffer[34] = 0x26;
  buffer[35] = Bit;
  buffer[36] = 0x29;
  buffer[37] = Bit;
  buffer[38] = 0x28;
  buffer[39] = Bit;
  buffer[40] = 0x2B;
  buffer[41] = Bit;
  buffer[42] = 0x2A;
  buffer[43] = Bit;
  buffer[44] = 0x2D;
  buffer[45] = Bit;
  buffer[46] = 0x2C;
  buffer[47] = Bit;
  buffer[48] = 0x2F;
  buffer[49] = Bit;
  buffer[50] = 0x2E;
  buffer[51] = Bit;
  buffer[52] = 0x31;
  buffer[53] = Bit;
  buffer[54] = 0x30;
  buffer[55] = Bit;
  buffer[56] = 0x33;
  buffer[57] = Bit;
  buffer[58] = 0x32;
  buffer[59] = Bit;
  buffer[60] = 0x35;
  buffer[61] = Bit;
  buffer[62] = 0x34;
  buffer[63] = Bit;
  ftdi->Write(buffer, 64, &bytesWritten);
  ftdi->Read(&CntState[0], 32, &bytesRead);
  //genio -> readbuf(24);
  return 0;
}

// commented out due to heavy FormMain interaction
/*
void  Funkcije::MeasureSpectrum(char Bit, TObject *Sender, double alpha, double beta, double gamma)
{
  int timer = 0;
  const int range = 512;
  double energy;
  double level;
  unsigned short int Flag = 0;
  unsigned short int CntState[16];
  int Results[range];
  int Results2[range];
  double xval[range];
  int Result;
  int cnt = 0;
  int loop = 0;
  //FormMain -> Chart1->BottomAxis->Title->Caption = "ToT";
  //FormMain -> Chart1->LeftAxis->Title->Caption = "Hits";
 //FormMain -> Chart2->BottomAxis->Title->Caption = "Energy/V";
 //FormMain -> Chart2->LeftAxis->Title->Caption = "Hits";
  AnsiString time;
  for (int i = 0; i < range; i++)
    {
      Results[i] = 0;
      Results2[i] = 0;
      xval[i] = 0;
    }
  ResetFIFO(0xFF); //resets FIFO
  ResetFIFO(0);
  do
    {
      time = TDateTime::CurrentDateTime().FormatString("yyyymmdd_hhmmss");
      Flag = ReadFlag();
      timer++;
      Sleep(1000);
      if (time.SubString(13,2) == "73")
	{
	  ResetFIFO(0xFF);//this prevents garabage to be written into fifo
	  Sleep(100);
	  FormMain -> ButtonSlowBitsClick(Sender);
	  Sleep(1000);
	  FormMain -> ButtonSetSensorClick(Sender);
	  Sleep(1000);
	  FormMain -> ButtonSetConfigClick(Sender);
	  Sleep(1000);
	  ResetFIFO(0);
	  Sleep(100);
	  timer = 0;
	}
    if (Flag != 0)
      {
	ReadToTCounterState(&CntState[0]);
	cnt = cnt + 16;
	//FormMain -> Series1 -> Clear(); // no Series 1
	for (int i = 0; i < range;  i++)
	  {
        for (int j = 0; j < 16; j++)
        {
          if ((CntState[j] >= (i*2 - 0)) && (CntState[j] < (i*2 + 2)))
          {
            Results[i] = Results[i] + 1;
          }
        }
        FormMain -> Series1 -> AddXY(i*2, Results[i]);

      }

      FormMain -> Chart1 -> Repaint();
      //FormMain -> Chart2 -> Repaint();
      FormMain -> MemoResults -> Lines -> Add("Number of hits is " + IntToStr(cnt));
      ResetFIFO(0xFF);
      ResetFIFO(0);

      //if (timer == 3000){    //was 20 !!!!

      //timer++;
      }

    Application->ProcessMessages();
    }
  while ((cnt < 200000) & !MeasurementStopped);
  FormMain -> Series1 -> Clear();
  FormMain -> Chart1->BottomAxis->Title->Caption = "ToT";
  FormMain -> Chart1->LeftAxis->Title->Caption = "Hits";
  for(int i = 0; i < range; i++)
    {
      FormMain -> Series1 -> AddXY(i*2, Results[i]);
    }
 //FormMain -> Series3 -> Clear();
 //FormMain -> Chart2->BottomAxis->Title->Caption = "Energy/V";
 //FormMain -> Chart2->LeftAxis->Title->Caption = "Hits";
 for(int i = 0; i < range; i++)
 {
  //FormMain -> Series3 -> AddXY(xval[i], Results2[i]);
 }
}
*/
// commented out due to heavy FormMain interaction

DWORD  Funkcije::ReadFlag()  //ToT FIFO flag
{
  DWORD Flag = 0;
  unsigned char buffer[2];
  unsigned long bytesWritten;
  unsigned long bytesRead;
  unsigned char Bit;
  Bit = 0x01;
  buffer[0]=0x36;    //changed!!!
  buffer[1]=Bit;
  ftdi->Write(buffer, 2, &bytesWritten);
  ftdi->Read(&Flag, 1, &bytesRead);
  return Flag;
}



void  Funkcije::ThresholdScan( int scantype, double startvalue, double fixedvalue )
{
  double xaux[1000];
  double yaux[1000];
  //unsigned char delay; //for L1 operation
  //delay = FormMain -> EditDelay -> Text.ToInt();
  
  double sensitvity = 0.02; //stops when less than 2% of signals responded  
  float voltage = startvalue;  

  if (scantype == 1)       //hitbus, signal (chopper voltage) is decreased until there are no hits
    {
      int stat = 10; // Number of measurements per voltage
      SetAmplitude(startvalue);
      InitPatternHitbus();
      InitCounter();
      
      nmeas = 0; //resets nmeas counter
      double readvalue = 0;
      for(int i=0; i<100; i++)
	{
	  readvalue = 0;
	  voltage -= 0.001;
	  this->SetAmplitude(voltage);
	  this->sleep(100);
	  
	  for(int i=0; i < stat; i++)
	    {
	      InitCounter(); //this resets the hitbus counter
	      StartPattern(); // 
	      this->sleep(50); // sleep
	      int counterstate = ReadCounterState();
	      readvalue = readvalue + counterstate;
	      std::cout << counterstate << " ";
	    }
	  std::cout << std::endl;
	  readvalue = readvalue/(double)stat; // mean of several readings.
	  //FormMain -> MemoResults -> Lines -> Add("Number of counts @ " + FloatToStrF(voltage, ffExponent, 6, 3) + " ist " + readvalue);
	  std::cout << "Number of counts at " << voltage << "V is " << readvalue << std::endl;
	  readvalue = readvalue/128.; // 128. pulses from the hitbus pattern InitPatternHitbus
	  
	  xaux[nmeas] = voltage;
	  yaux[nmeas] = readvalue;
	  nmeas++;
	}
      //while (readvalue > sensitvity);
      //FormMain -> Chart1->BottomAxis->Title->Caption = "Input/V";
      for(int i = 0; i<nmeas; i++)
	{
	  xmeas[nmeas-i-1] = xaux[i];
	  ymeas[nmeas-i-1] = yaux[i];
	}
    }
  if (scantype == 2)       //l1
    {
      int stat = 10;
      SetAmplitude(startvalue);
      InitPatternL1(Edit1,Edit2);
      nmeas = 0; //resets nmeas counter
      double readvalue = 0;
      do
	{
	  readvalue = 0;
	  voltage -= 0.01;
	  SetAmplitude(voltage);
	  sleep(50); // sleep
	  for(int i=0; i < stat; i++)
	    {
	      //InitCounter(); //this resets the hitbus counter
	      //StartPattern();
	      //readvalue = readvalue + DoManyRO(FormMain->EditColl -> Text.ToInt(), FormMain->EditRow -> Text.ToInt());
	      this->sleep(50); // sleep
	      readvalue = readvalue + DoManyRO(EditColl,EditRow);
	    }
	  readvalue = readvalue/stat;
	  //FormMain -> MemoResults -> Lines -> Add("Number of counts @ " + FloatToStrF(voltage, ffExponent, 6, 3) + " ist " + readvalue);
	  std::cout << "Number of counts at " << voltage << "V is " << readvalue << std::endl;
	  readvalue = readvalue/64.;
	  xaux[nmeas] = voltage;
	  yaux[nmeas] = readvalue;
	  nmeas++;
	}
      while (readvalue > sensitvity);
      //FormMain -> Chart1->BottomAxis->Title->Caption = "Input/V";
      for(int i = 0; i<nmeas; i++)
	{
	  xmeas[nmeas-i-1] = xaux[i];
	  ymeas[nmeas-i-1] = yaux[i];
	}
    }
}


void  Funkcije::SetAmplitude(double Injection)
{
  int volt = floor(16383 * Injection/1.8);
  std::cout << volt << " " << std::flush;
  SendDACPCB(volt);
  SendBuffer();
}


void  Funkcije::GetAllResults(signed int sign)
{
 if(nmeas > 5){

   // FormMain -> MemoResults -> Lines -> Add("Mean value is " + FloatToStrF(muvalue, ffExponent, 6, 5) + " (" + FloatToStrF(mustart, ffExponent, 3, 2) + ") ");
   std::cout << "Mean value is " << muvalue << " (" << mustart << ") " << std::endl;
   // FormMain -> MemoResults -> Lines -> Add("Sigma is " + FloatToStrF(sigvalue, ffExponent, 6, 5) + " (" + FloatToStrF(sigstart, ffExponent, 3, 2) + ") " );
   std::cout << "Sigma is " << sigvalue << " (" << sigstart << ") " << std::endl;
  if (sign ==1) 
    {
      //double mue = (1666*muvalue/0.185);//sens cali
      //double mue = (6200*muvalue/0.275);//Cd cali
      double mue = 1660*(muvalue)*0.1481*0.59*(1/0.11);//test pixel cali    factor 0.59 generates the equivalent pos inj (amplitude) from neg inj - 0.94mv amplitude is Fe55
      //FormMain -> MemoResults -> Lines -> Add( "Mean value in e is " + FloatToStrF(mue, ffExponent, 5, 4) + " e (C).");
      std::cout << "Mean value in electrons is " << mue << "e (C)." << std::endl;
    }
  if (sign ==1) 
    {
      //double sige = (1666*sigvalue/0.185);//sens cali
      //double sige = (6200*sigvalue/0.275);//Cd cali
      double sige = 1660*(sigvalue)*0.1481*0.59*(1/0.11);//test pixel cali   0.59 is mean value
      //FormMain -> MemoResults -> Lines -> Add("Sigma in e is " + FloatToStrF(sige, ffExponent, 5, 4) + " e (C).");
      std::cout << "Sigma in e is " << sige << " e (C)." << std::endl;
    }
  double step = 0;
  double errf;
  if (sign > 0)  
    errf = 0;
  if (sign <= 0)  
    errf = 1;
  double finex;
  step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
  double lowerx;
  lowerx = muvalue - 6 * sigvalue;
  if (xmeas[0] > lowerx)
    {
      finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
      do
	{
	  if (sign > 0) 
	    errf = errf + step * Gauss(finex, muvalue, sigvalue);
	  if (sign <= 0) 
	    errf = errf - step * Gauss(finex, muvalue, sigvalue);
	  finex = finex + step;
	}
      while (finex < xmeas[0]);
    }
  else 
    finex = xmeas[0];
  for(int i = 0; i < nmeas; i++)
    {
      yfit[i] = errf;
      for(int j = 0; j < INT_SENS; j++)
	{
	  if (sign > 0) 
	    errf = errf + step * Gauss(finex, muvalue, sigvalue);
	  if (sign <= 0) 
	    errf = errf - step * Gauss(finex, muvalue, sigvalue);
	  finex = finex + step;
	}
    }
  /*
  for(int i = 0; i < nmeas; i++)
    {
      FormMain -> Series1 -> AddXY(xmeas[i],ymeas[i]);
      FormMain -> Series2 -> AddXY(xmeas[i],yfit[i]);
    }
  */
  //FormMain -> MemoResults -> Lines -> Add("Number of iterations is " + IntToStr(iteration));
  std::cout << "Number of iterations is " << iteration << std::endl;
 }
 else 
   {
     //FormMain -> MemoResults -> Lines -> Add("too few points");
     std::cout << "Too few points." << std::endl;
   }
}

double  Funkcije::Gauss(double x, double mu, double sig)  //gauss function
{
  return 1/(sig * sqrt(2*PI))*exp( - (x - mu)*(x - mu)/(2 * sig * sig));
}

double  Funkcije::DGaussDmu(double x, double mu, double sig) //1st derivation
{
  return 2 * (x - mu)/(2 * sig * sig) * Gauss(x, mu, sig);
}

double  Funkcije::DGaussDsig(double x, double mu, double sig) //1st derivation
{
  return -1 * Gauss(x, mu, sig) / (sig)  +  2 * (x - mu)*(x - mu) * Gauss(x, mu, sig) / (2 * sig * sig * sig);
}

long double  Funkcije::ChiSquare(double mu, double sig, signed int sign) //chi square
{
  double step = 0;
  double errf;
  if (sign >= 0) 
    errf = 0;
  if (sign <= 0) 
    errf = 1;
  double chisq = 0;
  double finex;
  step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
  double lowerx;
  lowerx = mu - 6 * sig;
  if (xmeas[0] > lowerx)
    {
      finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
      do
	{
	  if (sign > 0) 
	    errf = errf + step * Gauss(finex, muvalue, sigvalue);
	  if (sign <= 0) 
	    errf = errf - step * Gauss(finex, muvalue, sigvalue);
	  finex = finex + step;
	}
      while (finex < xmeas[0]);
    }
 else 
   finex = xmeas[0];
 for(int i = 0; i < nmeas; i++)
   {
     chisq = chisq + (ymeas[i] - errf) * (ymeas[i] - errf);
     for(int j = 0; j < INT_SENS; j++)
       {
	 if (sign >= 0) 
	   errf = errf + step * Gauss(finex, mu, sig);
	 if (sign <= 0) 
	   errf = errf - step * Gauss(finex, mu, sig);
	 finex = finex + step;
       }
   }
 return chisq;
}

long double  Funkcije::D2_ChiSquareAna(int index1, int index2, double mu, double sig, signed int sign)  //2nd derivation of chi2
{
  double d_chisq = 0;
  if ((index1 == 1) & (index2 == 1))
    {
      double step = 0;
      double d_errf = 0;
      //double d_chisq = 0;
      double finex;
      step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
      double lowerx;
      lowerx = mu - 6 * sig;
      if (xmeas[0] > lowerx)
	{
	  finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
	  do
	    {
	      if (sign > 0) 
		d_errf = d_errf + step * DGaussDmu(finex, muvalue, sigvalue);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDmu(finex, muvalue, sigvalue);
	      finex = finex + step;
	    }
	  while (finex < xmeas[0]);
	}
      else 
	finex = xmeas[0];

      for(int i = 0; i < nmeas; i++)
	{
	  d_chisq = d_chisq + d_errf * d_errf;
	  for(int j = 0; j < INT_SENS; j++)
	    {
	      if (sign >= 0) 
		d_errf = d_errf + step * DGaussDmu(finex, mu, sig);
      if (sign <= 0) 
	d_errf = d_errf - step * DGaussDmu(finex, mu, sig);
      finex = finex + step;
	    }
	}
    };
  if ((index1 == 2) & (index2 == 2))
    {
      double step = 0;
      double d_errf = 0;
      double finex;
      step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
      double lowerx;
      lowerx = mu - 6 * sig;
      if (xmeas[0] > lowerx)
	{
	  finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
	  do
	    {
	      if (sign > 0) 
		d_errf = d_errf + step * DGaussDsig(finex, muvalue, sigvalue);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDsig(finex, muvalue, sigvalue);
	      finex = finex + step;
	    }
	  while (finex < xmeas[0]);
	}
      else 
	finex = xmeas[0];
  for(int i = 0; i < nmeas; i++)
    {
      d_chisq = d_chisq + d_errf * d_errf;
      for(int j = 0; j < INT_SENS; j++)
	{
	  if (sign >= 0) 
	    d_errf = d_errf + step * DGaussDsig(finex, mu, sig);
	  if (sign <= 0) 
	    d_errf = d_errf - step * DGaussDsig(finex, mu, sig);
	  finex = finex + step;
	}
    }
    };
  if (((index1 == 1) & (index2 == 2)) | ((index1 == 2) & (index2 == 1)))
    {
      double step = 0;
      double d1_errf = 0;
      double d2_errf = 0;
      double finex;
      step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
      double lowerx;
      lowerx = mu - 6 * sig;
      if (xmeas[0] > lowerx)
	{
	  finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
	  do
	    {
	      if (sign >= 0) 
		d1_errf = d1_errf + step * DGaussDmu(finex, mu, sig);
	      if (sign <= 0) 
		d1_errf = d1_errf - step * DGaussDmu(finex, mu, sig);
	      if (sign >= 0) 
		d2_errf = d2_errf + step * DGaussDsig(finex, mu, sig);
	      if (sign <= 0) 
		d2_errf = d2_errf - step * DGaussDsig(finex, mu, sig);;
	      finex = finex + step;
	    }
	  while (finex < xmeas[0]);
	}
      else 
	finex = xmeas[0];
      for(int i = 0; i < nmeas; i++)
	{
	  d_chisq = d_chisq + d1_errf * d2_errf;
	  for(int j = 0; j < INT_SENS; j++)
	    {
	      if (sign >= 0) 
		d1_errf = d1_errf + step * DGaussDmu(finex, mu, sig);
	      if (sign <= 0) 
		d1_errf = d1_errf - step * DGaussDmu(finex, mu, sig);
	      if (sign >= 0) 
		d2_errf = d2_errf + step * DGaussDsig(finex, mu, sig);
	      if (sign <= 0) 
		d2_errf = d2_errf - step * DGaussDsig(finex, mu, sig);
	      finex = finex + step;
	    }
	}
    };
  return d_chisq;
}

long double  Funkcije::D_ChiSquareAna(int index, double mu, double sig, signed int sign)  //1st derivation of chi2
{
  double d_chisq = 0;
  if (index == 1)
    {
      double step = 0;
      double d_errf = 0;
      double errf;
      if (sign >= 0) errf = 0;
      if (sign <= 0) errf = 1;
      double finex;
      step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
      double lowerx;
      lowerx = mu - 6 * sig;
      if (xmeas[0] > lowerx)
	{
	  finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
	  do
	    {
	      if (sign >= 0) 
		errf = errf + step * Gauss(finex, mu, sig);
	      if (sign <= 0) 
		errf = errf - step * Gauss(finex, mu, sig);
	      if (sign >= 0) 
		d_errf = d_errf + step * DGaussDmu(finex, mu, sig);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDmu(finex, mu, sig);
	      finex = finex + step;
	    }
	  while (finex < xmeas[0]);
	}
      else 
	finex = xmeas[0];
      for(int i = 0; i < nmeas; i++)
	{
	  d_chisq = d_chisq + (ymeas[i] - errf) * d_errf;
	  for(int j = 0; j < INT_SENS; j++)
	    {
	      if (sign >= 0) 
		errf = errf + step * Gauss(finex, mu, sig);
	      if (sign <= 0) 
		errf = errf - step * Gauss(finex, mu, sig);
	      if (sign >= 0) 
		d_errf = d_errf + step * DGaussDmu(finex, mu, sig);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDmu(finex, mu, sig);
	      finex = finex + step;
	    }
	}
    };
  if (index == 2)
    {
      double step = 0;
      double d_errf = 0;
      double errf;
      if (sign >= 0) 
	errf = 0;
      if (sign <= 0) 
	errf = 1;
      double finex;
      step = (xmeas[nmeas-1] -  xmeas[0])/((nmeas-1) * INT_SENS);
      double lowerx;
      lowerx = mu - 6 * sig;
      if (xmeas[0] > lowerx)
	{
	  finex = xmeas[0] - ceil((xmeas[0] - lowerx)/step) * step;
	  do
	    {
	      if (sign >= 0) 
		errf = errf + step * Gauss(finex, mu, sig);
	      if (sign <= 0) 
		errf = errf - step * Gauss(finex, mu, sig);
	      if (sign >= 0) 
		d_errf = d_errf + step * DGaussDsig(finex, mu, sig);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDsig(finex, mu, sig);
	      finex = finex + step;
	    }
	  while (finex < xmeas[0]);
	}
      else 
	finex = xmeas[0];
      for(int i = 0; i < nmeas; i++)
	{
	  d_chisq = d_chisq + (ymeas[i] - errf) * d_errf;
	  for(int j = 0; j < INT_SENS; j++)
	    {
	      if (sign >= 0) 
		errf = errf + step * Gauss(finex, mu, sig);
	      if (sign <= 0) 
		errf = errf - step * Gauss(finex, mu, sig);
	      if (sign >= 0) 
		d_errf = d_errf + step * DGaussDsig(finex, mu, sig);
	      if (sign <= 0) 
		d_errf = d_errf - step * DGaussDsig(finex, mu, sig);
	      finex = finex + step;
	    }
	}
    };
  return d_chisq;
}

int  Funkcije::FindClosest(double* array, double value, int length)  //finds the closest number in an array
{
  int closest = 0;
  double minimum;
  minimum = fabs(value - array[0]);
  for(int i = 1; i < length; i++)
    {
      if (fabs(value - array[i]) < minimum)
	{
	  minimum = fabs(value - array[i]);
	  closest = i;
	}
    }
  return closest;
}

void  Funkcije::FitLM(signed int sign)  //fit levenberg-marquart
{
  int indexhigh;
  int indexlow;
  int indexmiddle;
  if (nmeas > 5)
    {
      indexhigh = FindClosest(ymeas, 0.8, nmeas);
      indexlow = FindClosest(ymeas, 0.2, nmeas);
      indexmiddle = FindClosest(ymeas, 0.5, nmeas);
      mustart = xmeas[indexmiddle];
      sigstart = fabs(xmeas[indexhigh] - xmeas[indexlow])/2;
      if (sigstart == 0){
	sigstart = 0.001;
      }
      muvalue = mustart;
      sigvalue = sigstart;
      iteration = 0;
      int maxit = 10;
      long double lambda = 0.0001;
      double dmumu;
      double dsisi;
      double dmusi;
      double dmu;
      double dsi;
      double chisq;
      double delta;
      chisq = ChiSquare(muvalue, sigvalue, -1);
      do
	{
	  dmumu = (1 + lambda) * D2_ChiSquareAna(1, 1, muvalue, sigvalue, sign);
	  dsisi = (1 + lambda) * D2_ChiSquareAna(2, 2, muvalue, sigvalue, sign);
	  dmusi = D2_ChiSquareAna(1, 2, muvalue, sigvalue, sign);
	  dmu = D_ChiSquareAna(1, muvalue, sigvalue, sign);
	  dsi = D_ChiSquareAna(2, muvalue, sigvalue, sign);
	  //FormMain -> Series3 -> AddXY(iteration, dmu );
	  if ((dmumu * dsisi - dmusi * dmusi) != 0) 
	    muvalue = muvalue + ((dmu * dsisi - dsi * dmusi)/(dmumu * dsisi - dmusi * dmusi));
	  if ((dmumu * dsisi - dmusi * dmusi) != 0) 
	    sigvalue = sigvalue + ((dsi * dmumu - dmu * dmusi)/(dmumu * dsisi - dmusi * dmusi));
	  delta = ChiSquare(muvalue, sigvalue, sign) - chisq;
	  if (delta < 0)  
	    lambda = lambda/10;
	  else 
	    lambda = lambda * 10;
	  chisq = ChiSquare(muvalue, sigvalue, sign);
	  //FormMain -> Series3 -> AddXY(iteration, lambda );
	  //FormMain -> Series3 -> AddXY(iteration, dmumu );
	  iteration++;
	}
      while ( (iteration < maxit) );
    }
}

//fits are finished
/*
void  Funkcije::openOszi()
{
  Oszi         = new TGPIB_TDS(GPIB_Interface, ID_OSZI);
  Oszi -> Open();
}

void  Funkcije::SetOszi()
{
   GPIB_Interface -> Send (1, "CH" + AnsiString(1) + ":VOLT " + FloatToStrF(0.01, ffFixed, 6, 6)); //v pro div//was 0.05  25mvfine
   GPIB_Interface -> Send (1, "TRIGGER:A:COMM:SOURCE CH" + AnsiString(1));  //!!!
   //Oszi -> Send ("TRIGGER:A:COMM:SOURCE EXT");
   //GPIB_Interface -> Send (1, "TRIGGER:A:LEV 0.010");//!!!! //10mv
   //Oszi -> Send ("TRIGGER:A:FORCE");
   //Oszi -> Send ("TRIGGER:A:MODE NORMAL"); //???
   GPIB_Interface -> Send (1, "SELECT:CH" + AnsiString(1)+ " ON");   //turns ch on
   //Oszi -> Send ("CH" + AnsiString(1)+ ":VOLTS 0.5" );
   //Oszi -> Send ("MEASU:IMM:SOURCE CH" + AnsiString(1));//source for measurement
   //Oszi -> Send ("MEASU:IMM:TYP AMP" );
   //Oszi -> Send ("MEASU:IMM:TYP MAX" );
   //Oszi -> Send ("ACQUIRE:STOPA SEQUENCE" );
   //Oszi -> Send ("ACQUIRE:STATE ON" );
   //Oszi -> Send ("ACQUIRE:STATE STOP" );
   //Oszi -> Send ("ACQUIRE:STATE RUN" );
   //GPIB_Interface -> Send (1, "CH1:POSITION 0.0" );
   GPIB_Interface -> Send (1, "CH1:POSITION -4.0" );
   //GPIB_Interface -> Send (1, "HORIZONTAL:POSITION -2.0" );
   //Oszi -> Send ("CH1:IMP FIFTY" );
   GPIB_Interface -> Send (1, "CH1:IMP MEG" );
   GPIB_Interface -> Send (1, "CH1:COUPL AC" );
   GPIB_Interface -> Send (1, "CH1:BAND TWENTY" );
   //AnsiString ValStr = Oszi->SendAndReceive ("MEASU:IMM:VALUE?" );
   //FormMain->MemoIvan->Lines->Add("Messung Amplitude:" + ValStr);
   //float readvalue = ValStr.ToDouble();
   //FormMain->Series1->AddXY(freq, readvalue, "", clTeeColor);
   //GPIB_Interface->Send (1, "HORIZONTAL:SCALE 2E-6"); //t pro div  was 2us
   //Oszi -> Send ("ACQUIRE:STATE STOP" );
   //Oszi -> Send ("ACQUIRE:STATE OFF" );
   GPIB_Interface -> Send (1, "MEASU:IMM:SOURCE CH" + AnsiString(1));//source for measurement
   GPIB_Interface -> Send (1, "MEASU:IMM:TYP MAX" );
   //GPIB_Interface -> Send (1, "MEASU:IMM:TYP PK2PK" );
   //GPIB_Interface -> Send (1, "MEASU:IMM:TYP AREA" );
   //GPIB_Interface -> Send (1, "MEASU:IMM:TYP RMS" );
   //GPIB_Interface -> Send (1, "MEASU:IMM:TYP AMP" );
   GPIB_Interface -> Send (1, "MEASU:IMM:REFLEVEL:METHOD ABSOLUTE" );
   GPIB_Interface -> Send (1, "MEASU:IMM:REFLEVEL:ABS:HIGH 40E-3" );
   GPIB_Interface -> Send (1, "MEASU:IMM:REFLEVEL:ABS:LOW 20E-3" );
   GPIB_Interface -> Send (1, "MEASU:IMM:REFLEVEL:ABS:MID 30E-3" );
   GPIB_Interface -> Send (1, "MEASU:IMM:REFL:ABS:MID2 30E-3" );
   //GPIB_Interface -> Send (1, "MEASU:IMM:TYP PWIDTH" );
   GPIB_Interface -> Send (1, "ACQUIRE:STOPA SEQUENCE" ); //!!!!!!   stop at sequence
   GPIB_Interface -> Send (1, "TRIGGER:A:MODE NORMAL");
   Sleep(1000);

}

void  Funkcije::SetOsziSmall()
{

   GPIB_Interface -> Send (1, "RST" );
   GPIB_Interface->Send (1, "HORIZONTAL:SCALE 10E-6"); //t pro div

   Sleep(1000);

}
double  Funkcije::ReadOszi()
{
   TObject *Sender;
   GPIB_Interface -> Send (1, "ACQUIRE:STATE RUN" );
   //GPIB_Interface -> Send (1, "TRIGGER:A:MODE AUTO"); //!!!!! noise !!!!
   //GPIB_Interface -> Send (1, "ACQUIRE:STATE ON" );  //!!!!!
   Sleep(500);//2s
   //FormMain -> ButtonStartSequencerClick(Sender);  //!!!!!use this for cali
   //SetStrobeSlow(0xFF); //strobe off
   //SetStrobeSlow(0x00); //strobe off
   //SendBuffer();
   //GPIB_Interface -> Send (1, "TRIGGER:A:FORCE");  //!!!!! noise
   Sleep(500);//was 100
   AnsiString ValStr = GPIB_Interface->SendAndReceive (1, "MEASU:IMM:VALUE?" );
   FormMain -> MemoResults -> Lines -> Add(ValStr);
   float readvalue = ValStr.ToDouble();
   return readvalue;
   //if(2*readvalue*20E-3/2E-6 >= 0)
   //return sqrt(2*readvalue*20E-3/2E-6);
   //else return 0;
   //if(2*readvalue*190E-3/10E-6 >= 0)
   //return sqrt(2*readvalue*190E-3/10E-6);
   //else return 0;
   //GPIB_Interface -> Send (1, "ACQUIRE:STATE OFF" );
}

void  Funkcije::openGPIB()

{
 GPIB_Interface = new TGPIB_Interface_KEITHLEY_USB(FormMain -> MemoIvan -> Lines);
 //GPIB_Interface = new TGPIB_Interface(FormMain -> MemoIvan -> Lines);
 GPIB_Interface -> SendDebugTo(FormMain -> MemoIvan -> Lines);
 GPIB_Interface -> NewDevice (1);//THIS WAS NECESSARY
}
*/
// commented out
// NO GPIB usage

unsigned char  Funkcije::Readout(int Coll, int Row)  //starts pattern for hitbus
{

 //InitCounterDelayC(FormMain->Edit1->Text.ToInt(),FormMain->Edit2->Text.ToInt());//sets delay
 //unsigned char Bit = 0xFF; //gen 7 (resets delay counters)
 //genio->addpair( 0x78, Bit );
 //Bit = 0xFF - 0x80;       //gen 7 (resets delay counters)
 //genio->addpair( 0x78, Bit );
 //SetStrobeSlow(0xFF); //strobe


 //(bool Load, bool NextRow, bool SIn)


  for (int icnt = 0; icnt < 32; icnt ++) 
    {
      SendRoClock(false, true, false);//this clears the Y register
    }

  SendRoClock(false, true, true);//this clears the Y register
 //SendBuffer();
  
  for (int irow = 0; irow < 30; irow ++)
    {
      for (int icoll = 29; icoll >= 0; icoll--)
	{
	  if ( ( icoll == Coll)&&(irow == Row) )
	    {
	      if (icoll == 29) 
		SendRoClock(true, true, false);
	      else 
		SendRoClock(true, false, false);
	    }
	  else 
	    {
	      if (icoll == 29) 
		SendRoClock(false, true, false);
	      else 
		SendRoClock(false, false, false);
	    }
	}
    }
  
 //for (int icnt = 0; icnt < 32; icnt ++) {
   //SendRoClock(false, true, false);//this clears the Y register
 //}

  //  unsigned char  pattern = FormMain->Bit;
  unsigned char  pattern = this->Bit;
  //SetStrobeSlow(0x00); //strobe off
  //SendBuffer();
  pattern = pattern | PAT_Reset; //reset pixel
  genio->addpair( 0x41, pattern);
  genio->addpair( 0x41, pattern);
  genio->addpair( 0x41, pattern);
  genio->addpair( 0x41, pattern);
  //pattern = FormMain->Bit;
  pattern = this->Bit;
  genio->addpair( 0x41, pattern);
  SendBuffer();
  
  int Pixel;
  Pixel = ReadPixel(0x50);//change this address!!!!!!!!!!!!!!!!!!!!!   why
  //FormMain -> EditPixel -> Text = Pixel;
  return 1-Pixel;
}


void  Funkcije::ReadoutFIFO()  //starts pattern for hitbus
{

 //InitCounterDelayC(FormMain->Edit1->Text.ToInt(),FormMain->Edit2->Text.ToInt());//sets delay
 //unsigned char Bit = 0xFF; //gen 7 (resets delay counters)
 //genio->addpair( 0x78, Bit );
 //Bit = 0xFF - 0x80;       //gen 7 (resets delay counters)
 //genio->addpair( 0x78, Bit );
 //SetStrobeSlow(0xFF); //strobe


 //(bool Load, bool NextRow, bool SIn)

  //  int numberofpixels;
  ResetFifo(0xFF);
  ResetFifo(0x00);
  for (int icnt = 0; icnt < 32; icnt ++) 
    {
      SendRoClock(false, true, false);//this clears the Y register
    }

  SendRoClock(false, true, true);//this clears the Y register
 //SendBuffer();

 for (int irow = 0; irow < 30; irow ++)
   {
     for (int icoll = 29; icoll >= 0; icoll--)
       {
	 if (icoll == 29) SendRoClock(false, true, false);
	 if (icoll == 0) 
	   {
	     SendRoClock(false, false, false);
	     LoadFifo(0xFF);
	     LoadFifo(0x00);
	   }
       if ((icoll != 0)&&(icoll != 29)) 
	 SendRoClock(false, false, false);
       }
   }
 
 //for (int icnt = 0; icnt < 32; icnt ++) {
 //SendRoClock(false, true, false);//this clears the Y register
 //}
 
 //unsigned char  pattern = FormMain->Bit;
 unsigned char  pattern = this->Bit;
 //SetStrobeSlow(0x00); //strobe off
 //SendBuffer();
 pattern = pattern | PAT_Reset; //reset pixel
 genio->addpair( 0x41, pattern);
 genio->addpair( 0x41, pattern);
 genio->addpair( 0x41, pattern);
 genio->addpair( 0x41, pattern);
 // pattern = FormMain->Bit;
 pattern = this->Bit;
 genio->addpair( 0x41, pattern);

 SendBuffer();
}
/*
void  Funkcije::PlotPixel(int x, int y, int z, int norows)
{

  float wx = 30, wy = norows;
  //float wx = 4, wy = 4;
  float fx = FormMain -> Image1->Width  / wx;
  float fy = FormMain -> Image1->Height / wy;
  FormMain -> Image1->Canvas->Pixels[10][10] = clRed;
  FormMain -> Image1->Canvas->Brush->Style = bsSolid;
  DWORD mcolor = (z << 16) + (z << 8) + z;
  FormMain -> Image1->Canvas->Brush->Color = mcolor;
  TRect r;
  r.Left = x*fx;
  r.Top = y*fy;
  r.Right = r.left + fx;
  r.Bottom = r.top + fy;
  FormMain -> Image1->Canvas->FillRect( r );
  //FormMain -> Image1-> Repaint();
}
*/

void  Funkcije::InitCounterDelayC(unsigned char InpBit, unsigned char InpBit2) //initiates injection - L1 delay!
{
  //  unsigned long bytesWritten;
  unsigned char Address;
  unsigned char Bit;
  Address = 0x58;     //delay2
  Bit = InpBit;
  genio->addpair( Address, Bit );
  Address = 0x5B;     //delay3
  Bit = InpBit2;
  genio->addpair( Address, Bit );
  genio->sendbuf();
}


int Funkcije::DoManyRO(int Coll, int Row)
{
 //InitPatternL1(i,Edit2->Text.ToInt());
 int cnt = 0;
 for(int i=0; i<64; i++)
   {
     StartPatternL1();
     cnt = cnt + Readout(Coll, Row);
   }
 return cnt;
}


double  Funkcije::FindInput(double input, int mode)
{

   //InitCounter();
   float voltage = input;
   double readvalue = 0;
   double initstep;
   initstep = 0.04;
   int maxit = 20;
   int noit = 0;
   double step;
   bool error = false;
   bool finish = false;
   //unsigned char delay;
   bool thrtoolow = false;
   //delay = FormMain -> Edit1 -> Text.ToInt();
   //delay = this->Edit1; // not used
   SetAmplitude(voltage);
   this->sleep(25);//25 // sleep

   if (mode == 1) 
     {
       InitCounter();
       //InitPatternL1(FormMain->Edit1->Text.ToInt(),FormMain->Edit2->Text.ToInt());
       InitPatternL1(Edit1,Edit2);
       //StartPatternMatrix();
       //Sleep(25);
     }
   if (mode == 2) 
     {

     }
   if (mode == 1) 
     {
       //readvalue = DoManyRO(FormMain->EditColl -> Text.ToInt(), FormMain->EditRow -> Text.ToInt());
       readvalue = DoManyRO(EditColl,EditRow);
     }
   if (mode == 2) 
     {
     
     }
   //the next two ifs check whether are we below or above thr

   if (mode == 1) 
     {
       if ((readvalue < 32))
	 {
	   step = initstep;
	   thrtoolow = true;
	 }
       else
	 {
	   step = -initstep;
	 }
     }
   if (mode == 2) 
     {
       
     }
   //the next loop decreases/increases signal till threshold is exceeded

   do
     {
       noit++;
       voltage += step;
       SetAmplitude(voltage);
       this->sleep(25);//25 // sleep
       if (mode == 1) 
	 {
	   //InitCounter();
	   //InitPatternL1(delay);
	   //StartPatternMatrix();
	   this->sleep(25); // sleep
	   //readvalue = DoManyRO(FormMain->EditColl -> Text.ToInt(), FormMain->EditRow -> Text.ToInt());
	   readvalue = DoManyRO(EditColl,EditRow);
	   //FormMain -> MemoResults -> Lines -> Add("Number of counts @ " + FloatToStrF(voltage, ffExponent, 5, 4) + " ist " + readvalue);
	   std::cout << "Number of counts at " << voltage << " is " << readvalue << std::endl;
	   if ((thrtoolow == true) && (readvalue > 32)) 
	     {
	       finish = true;
	     }
	   if ((thrtoolow == false) && (readvalue < 32)) 
	     {
	       finish = true;
	     }
	 }
       if (mode == 2) 
	 {
	   
	 }
     }
   while ((finish == false) && (error == false) && (noit <= maxit));
 //binary search
   double delta;
   delta = initstep/2;
   noit = 0;

   do 
     {
       noit++;
     if (readvalue > 32)
       {
	 voltage = voltage - delta;
       }
     else 
       {
	 voltage = voltage + delta;
       }
     SetAmplitude(voltage);
     this->sleep(25); // sleep
     if (mode == 1) 
       {
	 //readvalue = DoManyRO(FormMain->EditColl -> Text.ToInt(), FormMain->EditRow -> Text.ToInt());
	 readvalue = DoManyRO(EditColl,EditRow);
     }
     if (mode == 2) 
       {
	 
       }
     //FormMain -> MemoResults -> Lines -> Add("Number of counts @ " + FloatToStrF(voltage, ffExponent, 5, 4) + " ist " + readvalue);
     std::cout << "Number of counts at " << voltage << " is " << readvalue << std::endl;
     delta = delta/2;
     }
   while (((readvalue < 28) || (readvalue > 36)) && (error == false) && (noit <= maxit));   // was 54,74
   if ((error == false)&&(noit <= maxit)) 
     {
       return voltage;
     }
   else
     {
       return input;
       // FormMain -> MemoResults -> Lines -> Add("error!!!");
       std::cout << "Error!" << std::endl;
     }
}
