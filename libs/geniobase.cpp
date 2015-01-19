//---------------------------------------------------------------------------

#include "geniobase.h"
#include <stdio.h>
#include <iostream>

//---------------------------------------------------------------------------


GenioBase::GenioBase( ) 
{
  //extfunc_writedata = NULL;
  //extfunc_readdata  = NULL;
  //extfunc_logmessage= NULL;
  this->ftdi = NULL;
  TXBufPos = 0;
  ReadBackCount = 0;
  RingRXBufSize = GenioBase_RingRXBufSize;
}

GenioBase::GenioBase(FTDI* extftdi) 
{
  TXBufPos = 0;
  ReadBackCount = 0;
  RingRXBufSize = GenioBase_RingRXBufSize;
  this->ftdi = extftdi;
}

GenioBase::~GenioBase()
{
  //delete ftdi; // this must be done in external call
}

bool GenioBase::initializeFtdi(FTDI* extftdi)
{
  this->ftdi = extftdi;
  if(this->ftdi == NULL)
    return false;
  else
    return true;
}

// helper functions - mainly from mkhelpers.cpp
/* std::string IntToHex0x( DWORD value, char digits ) {
    char conversion[20];
    sprintf( conversion, "0x%%0%dX", digits );
    char hex[20];
    sprintf( hex, conversion, value );
    std::string hexs( hex );
    return( hexs );
} */
std::string IntToHex( unsigned int value, char digits ) 
{
    char conversion[20];
    sprintf( conversion, "%%0%dX", digits );
    char hex[20];
    sprintf( hex, conversion, value );
    std::string hexs( hex );
    return( hexs );
}
std::string BufferToHex( unsigned char *Buffer, int BytesToWrite ) {
        std::stringstream textline;
        textline << "   ";
        int BytesWritten = 0;
        for ( int x = 0; x < BytesToWrite; x++ ) {
                textline << IntToHex(Buffer[x],2)+" ";
                BytesWritten++;
        }
        return textline.str();
}


bool GenioBase::addpair(unsigned char genio_regaddr, unsigned char genio_regval, int readbackcount) 
{
  if (addpair(genio_regaddr, genio_regval))
    {
      ReadBackCount+=readbackcount;
      return true;
    } 
  else  
    return false;
}
bool GenioBase::addpair(unsigned char genio_regaddr, unsigned char genio_regval)
{
  if ( TXBufPos > GenioBase_TXBufSize-2 )
    return false;
  TXBuf[TXBufPos++] = genio_regaddr;
  TXBuf[TXBufPos++] = genio_regval;
  return true;
}

bool GenioBase::sendbuf() 
{
  bool ftStatus = false;
  unsigned long int BytesWritten = 0;
  if (TXBufPos > 0 ) 
    {
      //bool ftStatus = extfunc_writedata( TXBuf, TXBufPos, &BytesWritten );
      ftStatus = ftdi->Write( (unsigned char*)TXBuf, TXBufPos, &BytesWritten );
    }
  //std::cout << "Written " << BytesWritten << " Bytes." << std::endl;
  TXBufPos = 0;
  return ftStatus;
}

int GenioBase::readbuf(  ) 
{
  return readbuf( ReadBackCount );
}

int GenioBase::readbuf( unsigned long int BytesToRead ) 
{
  return readbuf( RXBuf, BytesToRead );
}

int GenioBase::readbuf( unsigned char *buffer, unsigned long int BytesToRead ) 
{
  unsigned long int BytesRead = 0;

  bool ftStatus = ftdi->Read( buffer, BytesToRead, &BytesRead );
  if(ftStatus == false)
      std::cout << "Communication Error" << std::endl;
  //std::cout << "Read " << BytesRead << " Bytes." << std::endl;
  ReadBackCount = 0;
  return BytesRead;
}

bool GenioBase::getStatus(unsigned long *receive, unsigned long *transmit, unsigned long *eventstatus)
{
    return ftdi->getStatus(receive, transmit, eventstatus);
}

int GenioBase::ringbufnext() 
{
  //int PrevRXBufRdPos = RingRXBufRdPos; // not used variable
  if ( RingRXBufRdPos != RingRXBufWrPos )
    RingRXBufRdPos=(RingRXBufRdPos+1)%RingRXBufSize;
  return RingRXBufRdPos;
}

int GenioBase::ringbufbytes() 
{
  return (RingRXBufSize + RingRXBufWrPos - RingRXBufRdPos) % RingRXBufSize;
}

unsigned char GenioBase::ringbufthis() 
{
  return RingRXBuf[RingRXBufRdPos];
}

int GenioBase::ringreadbuf( ) 
{
  return ringreadbuf( ReadBackCount );
}

int GenioBase::ringreadbuf( unsigned long int readbytes ) 
{
  if (readbytes > RingRXBufSize) 
    {
      //extfunc_logmessage( &(std::string( "Too large Read Request!" )));
      std::cout << "Read request too large!" << std::endl;
      return 0;
    }
  
  unsigned long int bytesread;
  unsigned int NewWritePos = RingRXBufWrPos + readbytes;
  if ( NewWritePos > RingRXBufSize ) 
    { // split into two reads
      bytesread = readbuf( &RingRXBuf[RingRXBufWrPos], RingRXBufSize-RingRXBufWrPos );
      RingRXBufWrPos = 0;
      readbytes = NewWritePos - RingRXBufSize;
    }
  bytesread = readbuf( &RingRXBuf[RingRXBufWrPos], readbytes );
  RingRXBufWrPos += readbytes;
  ReadBackCount = 0;
  return bytesread;
}

