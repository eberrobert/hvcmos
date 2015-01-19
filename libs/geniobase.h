//---------------------------------------------------------------------------

#ifndef geniobaseH
#define geniobaseH

#include <string>
#include <sstream>
#include <map>
#include "ftdi.h"

//---------------------------------------------------------------------------

const int GenioBase_TXBufSize = 1024*128;
const int GenioBase_RXBufSize = 1024*128;
const int GenioBase_RingRXBufSize = 1024*256;

class GenioBase 
{
 private:
  // Handle for the FTDI class
  FTDI* ftdi; 
  // Write Debug information to cout if true.
  bool BufferLog; 
  // data structures

  unsigned char RingRXBuf[GenioBase_RingRXBufSize];
  unsigned int RingRXBufWrPos, RingRXBufRdPos;
  unsigned int RingRXBufSize;
 protected:
 public:

  // The transmit Buffer
  unsigned char TXBuf[GenioBase_TXBufSize];
  // The Read back buffer
  unsigned char RXBuf[GenioBase_RXBufSize];
  // The Transmit Buffer Position. This is number of bytes to be written.
  unsigned int TXBufPos;
  // The Number of bytes to be read from the FTDI chip. Can be set by addpair(uchar, uchar, int ReadBackCount).
  unsigned int ReadBackCount;
  // the same for the RX Ring Buffer

  /** Constructor without initializing the connection to the FTDI. Need to initialize before read/write usage. Call initializeFtdi(FTDI*) then.*/
  GenioBase();
  /** Constructor with an initialized FTDI class.
   * Requires a pointer to the FTDI object.
   */
  GenioBase(FTDI* extftdi);
  /** Destructor.
   * Does not delete anything.
   */
  ~GenioBase();
  // external functions used by GenioBase
  //bool (*extfunc_writedata) ( unsigned char*, long unsigned int, long unsigned int * );
  //bool (*extfunc_readdata) ( unsigned char*, long unsigned int, long unsigned int * );
  
  //void (*extfunc_logmessage) ( std::string *message );
  
  // properties
  /** Sets the option to write debug output to the console. Requires a boolean.
   *
   * TRUE for writing.
   * FALSE: default. Do not write to output.
   */
  void SetBufferLog(bool);
  
  // Initilizing ftdi read/write externally
  /** Hand over the FTDI object.
   * Requires a pointer to the FTDI class.
   * Return FALSE, if pointer IS NULL.
   * Else return TRUE.
   */
  bool initializeFtdi(FTDI*);
  
  // methods
  /** Add Pair of a register address and register value.
   * 
   * Use this function to read or write from or to any registers in the FPGA.
   * To read, the value of the register value is not important.
   * 
   * Requires the register address (unsigned char) and register value (unsigned char).
   *
   * Returns TRUE, if the buffer size is smaller than the maximum buffer size. 
   * If FALSE, a sendbuf() should be called.
   */
  bool addpair(unsigned char genio_regaddr, unsigned char genio_regval);
  
  /** Add Pair of a register address and register value with expected amount of bytes to be read from the chip.
   *
   * Use this function to use the simple readbuf() call. The number of bytes read back is set with readbackcount.
   * 
   * Requires the register address (unsigend char), the register value (unsigned char), the number of bytes, which should be read back from the chip.
   * The register value is not important.
   * Calls addpair(unsigned char, unsigned char)
   * 
   * Returns TRUE, if call of addpair(unsigned char, unsigned char) was successful.
   */
  bool addpair(unsigned char genio_regaddr, unsigned char genio_regval, int readbackcount );
  
  /** Send Buffer to the FPGA via FTDI chip.
   * 
   * This function calls FTDI::Write(void*, long unsigned int, long unsigned int*).
   * It sets TXBufPos = 0.
   */
  bool sendbuf();
  
  /**  Read Buffer Function.
   *
   * Returns readbuf(unsigned long int); The number of bytes expected is set by addpair(unsigned char, unsigned char, int ReadBackCount).
   * */
  int readbuf(); // reads number of bytes expected
  
  /**  Read Buffer Function.
   * 
   * Returns readbuf(unsigned char*, unsigned long int).
   * 
   * Requires the number of bytes to be read from the FPGA.
   */
  int readbuf(unsigned long int BytesToRead);
  
  /** Read Buffer Function
   *
   * This function actually calls FTDI::Read(void*, long unsigned int , long unsigned int*).
   *
   * Requires the pointer to the buffer, the bytes to be read into the buffer. Writes the number of bytes read into BytesRead.
   *
   * Returns the number of bytes actually read.
   */
  int readbuf(unsigned char *buffer, unsigned long int BytesToRead);
  
  /**
   * @brief getStatus fetches the status from the ftdi->getStatus.
   * @param receive returns the number of bytes in the ftdi to read.
   * @param transmit returns the number of bytes in the ftdi to write.
   * @param eventstatus says something about the event status.
   * @return true, if the communication with the FTDI chip was successful.
   */
  bool getStatus(unsigned long int* receive, unsigned long int* transmit, unsigned long int* eventstatus);

  /**
   * @brief getReadBuffer returns a pointer to the buffer holding the results from the readbuf call.
   * @return returns an unsigned char pointer.
   */
  unsigned char* getReadBuffer();


  // Functions to work with a ring read buffer. Good for BitBandMode? 
  int ringreadbuf();
  int ringreadbuf(unsigned long int readbytes);
  int ringbufbytes();// amount of bytes in ring buffer
  int ringbufnext(); // next ringbuffer read pos
  unsigned char ringbufthis(); // returns current byte (read pos) of ring buffer
};


#endif

