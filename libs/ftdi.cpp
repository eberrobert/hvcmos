
#include "ftdi.h"

#include <vector>
#include <time.h>

FTDI::FTDI(void)
{
	ftdiHandle = 0;
}

//---------------------------------------------------------------------

FTDI::~FTDI(void)
{
}

//---------------------------------------------------------------------------
bool FTDI::Open(void)
{
    bool rval = Open(0);
    return rval;
}

//---------------------------------------------------------------------------

bool FTDI::Open(int DeviceIndex)
{
    
  //pin_ptr<FT_HANDLE> p = &ftdiHandle;
  FT_HANDLE* p = &ftdiHandle;
  status = FT_Open(DeviceIndex, p);
  return FT_SUCCESS(status);
}


bool FTDI::OpenEx(void* arg1, unsigned long int flags)
{
    FT_HANDLE* p = &ftdiHandle;
    status = FT_OpenEx(arg1, flags, p);
    return FT_SUCCESS(status);
}

//---------------------------------------------------------------------------
void FTDI::sleep(int milliseconds)
{
  timespec tim, tim2;
  tim.tv_sec = 0;
  tim.tv_nsec = milliseconds * 1e6;
  nanosleep(&tim, &tim2); // sleep
}

//---------------------------------------------------------------------------
bool FTDI::Read(
		void* lpBuffer, //LPVOID lpBuffer,
		long unsigned int nBufferSize, // DWORD nBufferSize,
                long unsigned int* lpBytesReturned //     LPDWORD lpBytesReturned)
		)
{
    status = FT_Read(
                     ftdiHandle,
                     lpBuffer,
                     nBufferSize,
                     lpBytesReturned
                     );

    return FT_SUCCESS(status);
}



//---------------------------------------------------------------------------
bool FTDI::Write(
		 void* lpBuffer, //LPVOID lpBuffer,
		 long unsigned int nBufferSize, //DWORD nBufferSize,
		 long unsigned int* lpBytesWritten //LPDWORD lpBytesWritten)
		 )
{
   status = FT_Write(
                      ftdiHandle,
                      lpBuffer,
                      nBufferSize,
                      lpBytesWritten
                      );

    return FT_SUCCESS(status);
}

//---------------------------------------------------------------------------
bool FTDI::Close(void)
{
    status = FT_Close(ftdiHandle);

    return FT_SUCCESS(status);
}

//---------------------------------------------------------------------------
bool FTDI::resetDevice(void)
{
    status = FT_ResetDevice(ftdiHandle);

    return FT_SUCCESS(status);
}

//---------------------------------------------------------------------------
bool FTDI::setBitMode(UCHAR Mask, UCHAR Mode)
{
    status = FT_SetBitMode(ftdiHandle,Mask,Mode);

    return FT_SUCCESS(status);
}



bool FTDI::setBaudRate(DWORD BaudRate)
{
    status = FT_SetBaudRate(ftdiHandle, BaudRate);

    return FT_SUCCESS(status);
}

bool FTDI::getQueueStatus(long unsigned int* queueBytes)
{
  return FT_SUCCESS(FT_GetQueueStatus(ftdiHandle,queueBytes));
}


// Functions added by Robert Eber

bool FTDI::getStatus(DWORD* receive, DWORD* transmit, DWORD* eventstatus)
{
  return FT_SUCCESS(FT_GetStatus(ftdiHandle,receive,transmit,eventstatus));
}

bool FTDI::purge()
{
  return FT_SUCCESS(FT_Purge(ftdiHandle, FT_PURGE_RX | FT_PURGE_TX));
}

bool FTDI::setDataCharacteristics( // One could assign different values...
				  //unsigned char wordlength, 
				  //unsigned char stopbits, 
				  //unsigned char parity
				   )
{
  return FT_SUCCESS(FT_SetDataCharacteristics(ftdiHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE));
}

bool FTDI::setTimeouts(long unsigned int readTimeout, long unsigned int writeTimeout)
{
  return FT_SUCCESS(FT_SetTimeouts(ftdiHandle, readTimeout, writeTimeout));
}

bool FTDI::getBitMode(unsigned char* lpBuffer)
{
  return FT_SUCCESS(FT_GetBitMode(ftdiHandle, lpBuffer));
}

bool FTDI::getDeviceInfo(FT_DEVICE* Device, unsigned long int* ID, char* serialnumber, char* description)
{
    void* dummy;
    return FT_SUCCESS(FT_GetDeviceInfo(
        ftdiHandle,    //FT_HANDLE ftHandle,
        Device,       //FT_DEVICE *lpftDevice,
        ID,            //LPDWORD lpdwID,
        serialnumber,  //PCHAR SerialNumber,
        description,   //PCHAR Description,
        dummy          //LPVOID Dummy
        ));
}

bool FTDI::listDevices(void* arg1, void* arg2, unsigned long int Flags)
{
    return FT_SUCCESS(FT_ListDevices(
        arg1, //PVOID pArg1,
        arg2, //PVOID pArg2,
        Flags  //DWORD Flags
    ));
}

/*
int FTDI::FillBuffer(unsigned char* buffer)
{
  this->vBuffer.push_back(buffer);
  return vBuffer.size();
}

bool FTDI::SendBuffer(long unsigned int* bytesWritten)
{
  return Write(&vBuffer, vBuffer.size(), bytesWritten);
}
*/
