#ifndef CONFIG_H
#define CONFIG_H

#include<vector>
#include<string>

class PCBconfig
{
private:
    double inj;
    double th1;
    double th2;
    bool CheckRange(double voltage); // 0 <= voltage <= 3.3
public:
    PCBconfig();
    ~PCBconfig();

    /**
     * @brief SetInj Sets the injection voltage on the PCB
     * @param voltage is the voltage for injection on the board. 0V <= voltage <= 3.3V
     */
    void SetInj(double voltage);

    /**
     * @brief SetTh1 Sets the 1st Threshold on the PCB. This is the global Threshold Voltage.
     * @param th1 Threshold voltage to be set. 0V <= th1 <= 3.3V. Usually around 2V.
     */
    void SetTh1(double th1);

    /**
     * @brief SetTh2 Sets the 2nd Threshold for the time walk compensated comparator.
     * @param th2 Threshold voltage for the 2nd comparator. 0V <= th2 <= 3.3V.
     */
    void SetTh2(double th2);

    /**
     * @brief GetInj returns the Injection Voltage stored in the configuration.
     * @return
     */
    double GetInj();

    /**
     * @brief GetTh1 returns the 1st Threshold Voltage stored in the configuration.
     * @return
     */
    double GetTh1();

    /**
     * @brief GetTh2 returns the 2nd Threshold Voltage stored in the configuration.
     * @return
     */
    double GetTh2();

};

class globalconfig
{
private:
    // 14 Bias DACs with SpareBits
    std::vector<bool> Spare;
    std::vector<int> bDAC;
    bool CheckRange(int i);
public:
    // Constructor and Destructor
    globalconfig();
    ~globalconfig();
    // Global configuration bits

    /**
     * @brief ABEn Holds the configuration for the Analog Buffers Enabled. True = off.
     */
    bool ABEn;

    /**
     * @brief CompOffNorm Holds the configuration for the normal Comparator Enabled.
     */
    bool CompOffNorm;

    /**
     * @brief CompOffB Holds the configuration for the time walk compensated comparator Enabled.
     */
    bool CompOffB;

    /**
     * @brief EnLowPass Enables the low pass filter if true.
     */
    bool EnLowPass;

    /**
     * @brief SetSpare Sets the Spare bit at position i.
     * @param i i'th Spare Bit
     * @param value boolean true or false for 1 or 0.
     */
    void SetSpare(int i, bool value);

    /**
     * @brief GetSpare returns the Spare bit as boolean at position i.
     * @param i i'th position for the Spare bit.
     * @return boolean true or false for SpareBit 1 or 0.
     */
    bool GetSpare(int i);

    /**
     * @brief SetbDAC Sets the BiasDAC at position i.
     * @param i i'th position of the BiasDAC.
     * @param value integer between 0 and 63 (6-bit).
     */
    void SetbDAC(int i, int value);

    /**
     * @brief GetbDAC returns the BiasDAC value at position i.
     * @param i i'th position of the BiasDAC.
     * @return integer value of the BiasDAC.
     */
    int GetbDAC(int i);




};

struct singlepixel
{
    bool DigInjEn;
    bool HBEn;
    bool Ld;
    bool AnaInj;
    int tdac1;
    int tdac2;
    int tdac3;
};

class pixelconfig
{
private:
    std::vector<bool> DigInjEn;
    std::vector<bool> HBEn;
    std::vector<bool> Ld;
    std::vector<bool> AnaInj;
    int rows;
    int columns;
    std::vector<int> tdac1; // to save TDAC values for each pixel
    std::vector<int> tdac2;
    std::vector<int> tdac3;
    //count internally as pixelnumbers.

public:

    /**
     * @brief pixelconfig Default constructor of this class.
     */
    pixelconfig();

    /**
     * @brief pixelconfig Constructor with the pixel layout configuration. Requires the number of pixel rows and columns.
     * @param rows: Number of pixel rows.
     * @param columns: Number of pixel columns.
     */
    pixelconfig(int rows, int columns);

    /**
      * Default destructor
      */
    ~pixelconfig();


    /**
     * @brief SetLayout Sets the number of rows and columns of the pixel layout.
     * @param rows: Number of pixel rows.
     * @param columns: Number of pixel columns.
     */
    void SetLayout(int rows, int columns); // if default constructor was called.

    /**
     * @brief SetRows just sets the number of rows of the pixel layout. Do not call alone. Can be used to resize the pattern. Requires the number of pixel rows.
     * @param rows. Number of pixel rows.
     */
    void SetRows(int rows);

    /**
     * @brief SetColumns Just sets the number of columns of the pixel layout. Do not call alone. Can be used to resize the pattern. Requires the number of pixel columns.
     * @param columns: Number of pixel columns.
     */
    void SetColumns(int columns);

    /**
     * @brief initialize Initializes the private vectors holding the pixel configuration with false. Number of pixels is the product of pixel rows times pixel columns.
     * @param numberofpixels the number of pixels, product of rows*columns
     */
    void initialize(int numberofpixels);

    /**
     * @brief CheckRange checks, if the pixelnumber is in range of the pixel layout.
     * @param pixelnumber is the pixel address to be checked as integer. Starts at 0 to rows*columns-1
     * @return returns true if pixelnumber is in range.
     */
    bool CheckRange(int pixelnumber);

    /**
     * @brief CheckRange checks, whether row and column are in range of the pixel layout.
     * @param row is the row address of the pixel. Starts at 1.
     * @param column is the column address of the pixel. Starts at 1.
     * @return returns true if both parameters are legal.
     */
    bool CheckRange(int row, int column);

    /**
     * @brief DecAd DECode ADdress of a single pixel with a given row and column to an absolute address.
     * @param row is the row of the pixel. Starts at 1.
     * @param column is the column address of the pixel. Starts at 1.
     * @return returns the absolute address of the pixel.
     */
    int DecAd(int row, int column);

    /**
     * @brief SetAnaInj Set Analogue injection for the pixel, addressed by row and column.
     * @param row is the row address, starts at 1.
     * @param column is the column address, starts at 1.
     * @param config sets AnaInj to true or false.
     */
    void SetAnaInj(int row,int column, bool config);

    /**
     * @brief SetLd
     * @param row: row address, starts at 1.
     * @param column: column address, starts at 1.
     * @param config: Sets Ld to true or false.
     */
    void SetLd(int row, int column, bool config);

    /**
     * @brief SetHBEn Connects the comparators to the Hit busses.
     * @param row: Row address of the pixel, starts at 1.
     * @param column: Column address of the pixel, starts at 1.
     * @param config: Connects, if true.
     */
    void SetHBEn(int row, int column, bool config);

    /**
     * @brief SetDigInjEn Enables the digital injection into the pixel cell.
     * @param row: Row address of the pixel, starts at 1.
     * @param column: Column address of the pixel, starts at 1.
     * @param config: Enables digital injection for that pixel, if true.
     */
    void SetDigInjEn(int row, int column, bool config);

    void SetAnaInj(int pixelnumnber, bool config);
    void SetLd(int pixelnumber, bool config);
    void SetHBEn(int pixelnumber, bool config);
    void SetDigInjEn(int pixelnumber, bool config);

    //This sets all pixels to the config.
    void SetAnaInj(bool config);
    void SetLd(bool config);
    void SetHBEn(bool config);
    void SetDigInjEn(bool config);


    /**
     * @brief SetConfig sets the configuration for a single pixel given by pixelnumber.
     * @param pixelnumber is the absolute pixel number, starts at 0.
     * @param pixelconf is the configuration of the four bits given in the struct singlepixel.
     */
    void SetConfig(int pixelnumber, singlepixel pixelconf);

    /**
     * @brief SetConfig sets the configuration for a single pixel given row and column number.
     * @param row is the row of the pixel, starts at 1.
     * @param column is the column of the pixel address, starts at 1.
     * @param pixelconf is of the struct singlepixel and holds the four configuration bits.
     */
    void SetConfig(int row, int column, singlepixel pixelconf);

    /**
     * @brief useful for initialization.
     * @param pixelconf holds the four configuration bits in the struct singlepixel.
     */
    void SetConfig(singlepixel pixelconf);

    /**
     * @brief SetConfig sets the configuration for all pixels. Just handles the four configuration bits as four bools.
     * @param DigInj sets digital injection true or false.
     * @param HBEn connects the comparator hit busses if true.
     * @param Ld loads the TDAC values for the RAM in the pixel cells.
     * @param AnaInj  enables analog injection into pixel cell.
     */
    void SetConfig(bool DigInj, bool HBEn, bool Ld, bool AnaInj);

    /**
     * @brief GetBits returns the bits set for a particular pixel addressed by row and column.
     * @param row: pixel row address, starts at 1.
     * @param column: pixel column address, starts at 1.
     * @return returns the struct singlepixel with the four configuration bits.
     */
    singlepixel GetBits(int row, int column);

    /**
     * @brief GetBits returns the bits set for a particular pixel addressed by the absolute pixel number.
     * @param pixelnumber: the absolute pixelnumber, starts at 0.
     * @return returns the struct singlepixel with the four configuration bits.
     */
    singlepixel GetBits(int pixelnumber);

    /**
     * @brief GetPatternAnaInj formats a string with the bits shown in rows and columns as "1" or "0" matching true and false.
     * @return returns the formatted string with the pixel configuration for the analog injection.
     */
    std::string GetPatternAnaInj();

    /**
     * @brief GetPatternHBEn formats a string with the bit settings for the digital hitbus. "1" is true, "0" is false.
     * @return returns the formatted string with the pixel configuration for the hitbus enabled settings.
     */
    std::string GetPatternHBEn();

    /**
     * @brief GetPatternLd formats a string with the bit settings for the load configuration of the pixels.
     * @return returns a formatted string in rows and columns with the pixel configuration for the load pattern.
     */
    std::string GetPatternLd();

    /**
     * @brief GetPatternDigInj formats a string with the bit settings for the digital injection pattern of the pixels.
     * @return returns a formatted string in rows and columns with the pixel configuration for the digital injection pattern.
     */
    std::string GetPatternDigInj();

    /**
     * @brief size returns the number of pixels. rows*columns.
     * @return the total number of pixels.
     */
    int size();

    /**
     * @brief SetTdac sets 3 TDACs for the indexed pixel.
     * @param pixel is the number of the pixel, the TDACs are set for.
     * @param tdac1 is the value of TDAC 1.
     * @param tdac2 is the value of TDAC 2.
     * @param tadc3 is the value of TDAC 3.
     */
    void SetTdac(int pixel, int tdac1, int tdac2, int tadc3);

    /**
     * @brief SetTdac sets 3 TDACs for the pixel in row and column.
     * @param row is the pixel row.
     * @param column is the pixel column.
     * @param tdac1 is the value of TDAC 1.
     * @param tdac2 is the value of TDAC 2.
     * @param tdac3 is the value of TDAC 3.
     */
    void SetTdac(int row, int column, int tdac1, int tdac2, int tdac3);

    /**
     * @brief SetTdac sets 3 TDAC values for all pixels. Useful for initialization.
     * @param tdac1 sets TDAC 1.
     * @param tdac2 sets TDAC 2.
     * @param tdac3 sets TDAC 3.
     */
    void SetTdac(int tdac1, int tdac2, int tdac3);

    /**
     * @brief GetTdac1 fetches the TDAC 1 setting for the given pixel.
     * @param pixel is the absolute pixel address, starts at 0.
     * @return the TDAC1 is returned as an integer.
     */
    int GetTdac1(int pixel);

    /**
     * @brief GetTdac2 fetches the TDAC 2 setting for the given pixel.
     * @param pixel is the absolute pixel address, starts at 0.
     * @return the TDAC2 is returned as an integer.
     */
    int GetTdac2(int pixel);

    /**
     * @brief GetTdac3 fetches the TDAC 3 setting for the given pixel.
     * @param pixel is the absolute pixel address, starts at 0.
     * @return the TDAC3 is returned as an integer.
     */
    int GetTdac3(int pixel);

    /**
     * @brief GetTdac1 fetches the TDAC 1 setting for the pixel in the given row and column.
     * @param row is the pixel row, starting at 1.
     * @param column is the pixel column, starting at 1.
     * @return TDAC3 setting for the given pixel.
     */
    int GetTdac1(int row, int column);

    /**
     * @brief GetTdac2 fetches the TDAC 2 setting for the pixel in the given row and column.
     * @param row is the pixel row, starting at 1.
     * @param column is the pixel column, starting at 1.
     * @return returns the TDAC2 setting for the pixel.
     */
    int GetTdac2(int row, int column);

    /**
     * @brief GetTdac3 fetches the TDAC 3 setting for the pixel in the given row and column.
     * @param row is the pixel row, starting at 1.
     * @param column is the pixel column, starting at 1.
     * @return returns the TDAC3 setting for the pixel.
     */
    int GetTdac3(int row, int column);

    /**
     * @brief GetTdac fetches all TDAC settings for the given pixel.
     * @param pixel is the absolute pixel address, starting at 0.
     * @return returns all TDAC settings in the struct singlepixel.
     */
    singlepixel GetTdac(int pixel);

    /**
     * @brief GetTdac fetches all TDAC settings for the given pixel in row and column.
     * @param row is the pixel row, starting at 1.
     * @param column is the pixel column starting at 1.
     * @return returns all TDAC settings in the struct singlepixel.
     */
    singlepixel GetTdac(int row, int column);

    /**
     * @brief WriteConfig writes the pixel configuration to a file.
     * @filename is the file name of the configuration file to write.
     * @return returns true if writing to the file was successful.
     */
    bool WriteConfig(std::string filename = "H35config.ini");

    /**
     * @brief ReadConfig reads the pixel configuration from the given file.
     * @param filename is the file name of the configuration file to read.
     * @return returns the number of lines read.
     */
    int ReadConfig(std::string filename = "H35config.ini");


};

#endif // CONFIG_H
