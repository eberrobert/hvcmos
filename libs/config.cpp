#include"config.h"
#include<vector>
#include<fstream>

pixelconfig::pixelconfig()
{

}

pixelconfig::pixelconfig(int rows, int columns)
{
    this->SetLayout(rows, columns);
}

pixelconfig::~pixelconfig()
{

}

void pixelconfig::SetLayout(int rows, int columns)
{
    this->SetColumns(columns);
    this->SetRows(rows);
    this->initialize(rows*columns);
}

void pixelconfig::SetColumns(int columns)
{
    this->columns = columns;
}

void pixelconfig::SetRows(int rows)
{
    this->rows = rows;
}

void pixelconfig::initialize(int numberofpixels)
{
    this->DigInjEn.resize(numberofpixels);
    this->HBEn.resize(numberofpixels);
    this->Ld.resize(numberofpixels);
    this->AnaInj.resize(numberofpixels);
    this->tdac1.resize(numberofpixels);
    this->tdac2.resize(numberofpixels);
    this->tdac3.resize(numberofpixels);
    for(int i=0; i<numberofpixels; i++)
    {
        this->DigInjEn[i] = false;
        this->HBEn[i] = false;
        this->Ld[i] = false;
        this->AnaInj[i] = false;
        this->tdac1[i] = 0;
        this->tdac2[i] = 0;
        this->tdac3[i] = 0;
    }
}

bool pixelconfig::CheckRange(int pixelnumber)
{
    if(pixelnumber < (this->rows*this->columns))
        return true;
    else
        return false;
}

bool pixelconfig::CheckRange(int row, int column)
{
    if(row <= this->rows && row != 0 && column <= this->columns && column != 0)
        return true;
    else
        return false;
}

int pixelconfig::DecAd(int row, int column)
{
    return ((row-1)*this->columns + column - 1);
}

void pixelconfig::SetDigInjEn(int pixelnumber, bool config) // count from 0
{
    if(CheckRange(pixelnumber))
        this->DigInjEn[pixelnumber] = config;
}

void pixelconfig::SetDigInjEn(int row, int column, bool config)
{
    if(CheckRange(row,column))
        this->SetDigInjEn(DecAd(row, column), config); // decode to absoule address.
}

void pixelconfig::SetDigInjEn(bool config)
{
    for(unsigned int i=0; i<this->DigInjEn.size(); i++)
    {
        this->DigInjEn[i] = config;
    }
}

void pixelconfig::SetHBEn(int pixelnumber, bool config)
{
    if(CheckRange(pixelnumber))
        this->HBEn[pixelnumber] = config;
}

void pixelconfig::SetHBEn(int row, int column, bool config)
{
    if(CheckRange(row,column))
        this->SetHBEn(DecAd(row, column), config);
}

void pixelconfig::SetHBEn(bool config)
{
    for(unsigned int i=0; i<this->HBEn.size(); i++)
    {
        this->HBEn[i] = config;
    }
}

void pixelconfig::SetLd(int pixelnumber, bool config)
{
    if(CheckRange(pixelnumber))
        this->Ld[pixelnumber] = config;
}

void pixelconfig::SetLd(int row, int column, bool config)
{
    if(CheckRange(row,column))
        this->SetLd(DecAd(row, column), config);
}

void pixelconfig::SetLd(bool config)
{
    for(unsigned int i=0; i<this->Ld.size(); i++)
    {
        this->Ld[i] = config;
    }
}

void pixelconfig::SetAnaInj(int pixelnumber, bool config)
{
    if(CheckRange(pixelnumber))
        this->AnaInj[pixelnumber] = config;
}

void pixelconfig::SetAnaInj(int row, int column, bool config)
{
    if(CheckRange(row,column))
        this->SetAnaInj(DecAd(row, column), config);
}

void pixelconfig::SetAnaInj(bool config)
{
    for(unsigned int i=0; i<this->AnaInj.size(); i++)
    {
        this->AnaInj[i] = config;
    }
}

void pixelconfig::SetConfig(int pixelnumber, singlepixel pixelconf)
{
    if(CheckRange(pixelnumber))
    {
        this->DigInjEn[pixelnumber] = pixelconf.DigInjEn;
        this->Ld[pixelnumber] = pixelconf.Ld;
        this->HBEn[pixelnumber] = pixelconf.HBEn;
        this->AnaInj[pixelnumber] = pixelconf.AnaInj;
    }
}

void pixelconfig::SetConfig(int row, int column, singlepixel pixelconf)
{
    if(CheckRange(row,column))
        this->SetConfig(DecAd(row, column), pixelconf);
}

void pixelconfig::SetConfig(singlepixel pixelconf)
{
    for(unsigned int i=0; i<this->DigInjEn.size(); i++)
    {
        this->SetConfig(i,pixelconf);
    }
}

void pixelconfig::SetConfig(bool DigInj, bool HBEn, bool Ld, bool AnaInj)
{
    singlepixel pixelconf;
    pixelconf.AnaInj = AnaInj;
    pixelconf.Ld = Ld;
    pixelconf.HBEn = HBEn;
    pixelconf.DigInjEn = DigInj;
    for(unsigned int i=0; i<this->DigInjEn.size(); i++)
    {
        this->SetConfig(i,pixelconf);
    }
}

singlepixel pixelconfig::GetBits(int row, int column)
{
    singlepixel returnvalue;
    returnvalue.AnaInj = 0;
    returnvalue.DigInjEn = 0;
    returnvalue.HBEn = 0;
    returnvalue.Ld = 0;

    if(row > this->rows || column > this->columns)
        return returnvalue;
    else
    {
        returnvalue.AnaInj = this->AnaInj[DecAd(row, column)];
        returnvalue.DigInjEn = this->DigInjEn[DecAd(row, column)];
        returnvalue.HBEn = this->HBEn[DecAd(row, column)];
        returnvalue.Ld = this->Ld[DecAd(row, column)];
    }
    return returnvalue;
}

singlepixel pixelconfig::GetBits(int pixelnumber)
{
    singlepixel returnvalue;
    returnvalue.AnaInj = 0;
    returnvalue.DigInjEn = 0;
    returnvalue.HBEn = 0;
    returnvalue.Ld = 0;
    if(pixelnumber < 0 || pixelnumber > (this->columns*this->rows))
        return returnvalue;
    else
    {
        returnvalue.AnaInj = this->AnaInj[pixelnumber];
        returnvalue.DigInjEn = this->DigInjEn[pixelnumber];
        returnvalue.HBEn = this->HBEn[pixelnumber];
        returnvalue.Ld = this->Ld[pixelnumber];
    }
    return returnvalue;
}

std::string pixelconfig::GetPatternAnaInj()
{
    std::string pattern = "";
    for(unsigned int i=0; i<this->AnaInj.size(); i++)
    {
        if(i % (columns) == 0 && i > 1)
        {
            pattern += "\n";
        }
        if(this->AnaInj[i] == true)
            pattern += "1";
        else
            pattern += "0";
    }
    return pattern;
}

std::string pixelconfig::GetPatternDigInj()
{
    std::string pattern = "";
    for(unsigned int i=0; i<this->DigInjEn.size(); i++)
    {
        if(i % (columns) == 0  && i > 1)
        {
            pattern += "\n";
        }
        if(this->DigInjEn[i] == true)
            pattern += "1";
        else
            pattern += "0";
    }
    return pattern;
}

std::string pixelconfig::GetPatternHBEn()
{
    std::string pattern = "";
    for(unsigned int i=0; i<this->HBEn.size(); i++)
    {
        if(i % (columns) == 0 && i > 1)
        {
            pattern += "\n";
        }
        if(this->HBEn[i] == true)
            pattern += "1";
        else
            pattern += "0";
    }
    return pattern;
}

std::string pixelconfig::GetPatternLd()
{
    std::string pattern = "";
    for(unsigned int i=0; i<this->Ld.size(); i++)
    {
        if(i % (columns) == 0 && i > 1)
        {
            pattern += "\n";
        }
        if(this->Ld[i] == true)
            pattern += "1";
        else
            pattern += "0";
    }
    return pattern;
}

int pixelconfig::size()
{
    return (this->rows*this->columns);
}

void pixelconfig::SetTdac(int pixel, int tdac1, int tdac2, int tdac3)
{
    if(CheckRange(pixel))
    {
        this->tdac1[pixel] = tdac1;
        this->tdac2[pixel] = tdac2;
        this->tdac3[pixel] = tdac3;
    }
}

void pixelconfig::SetTdac(int row, int column, int tdac1, int tdac2, int tdac3)
{
    if(CheckRange(row,column))
    {
        SetTdac(this->DecAd(row,column), tdac1, tdac2, tdac3);
    }
}

void pixelconfig::SetTdac(int tdac1, int tdac2, int tdac3)
{
    for(int i=0; i<this->size(); i++)
    {
        SetTdac(i, tdac1, tdac2, tdac3);
    }
}

int pixelconfig::GetTdac1(int pixel)
{
    return this->tdac1[pixel];
}

int pixelconfig::GetTdac2(int pixel)
{
    return this->tdac2[pixel];
}

int pixelconfig::GetTdac3(int pixel)
{
    return this->tdac3[pixel];
}

int pixelconfig::GetTdac1(int row, int column)
{
    return GetTdac1(this->DecAd(row, column));
}

int pixelconfig::GetTdac2(int row, int column)
{
    return GetTdac2(this->DecAd(row, column));
}

int pixelconfig::GetTdac3(int row, int column)
{
    return GetTdac3(this->DecAd(row, column));
}

singlepixel pixelconfig::GetTdac(int pixel)
{
    singlepixel pix;
    pix.tdac1 = GetTdac1(pixel);
    pix.tdac2 = GetTdac2(pixel);
    pix.tdac3 = GetTdac3(pixel);
    return pix;
}

singlepixel pixelconfig::GetTdac(int row, int column)
{
    return GetTdac(this->DecAd(row, column));
}

bool pixelconfig::WriteConfig(std::string filename)
{
    // This is the version of the file configuration.
    int version = 1;
    std::ofstream file;
    bool success = false;
    file.open(filename.c_str());
    success = file.is_open();
    file << "#version " << version << "\n";
    for(int i=0; i<this->size(); i++)
    {
        file << GetTdac1(i) << " " << GetTdac2(i) << " " << GetTdac3(i) << "\n";
    }
    file.close();
    return success;
}

int pixelconfig::ReadConfig(std::string filename)
{
    std::ifstream file;
    file.open(filename.c_str());
    int i = 0;
    int version = 1;
    if(version == 1)
    {
        int tdac1, tdac2, tdac3;
        while(file.good())
        {
            file >> tdac1 >> tdac2 >> tdac3;
            SetTdac(i, tdac1, tdac2, tdac3);
            i++;
            if(i > size())
             break;
        }
    }
    file.close();
    return i;
}



// -------------------------------------------------------------------
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Class globalconfig
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// -------------------------------------------------------------------
globalconfig::globalconfig()
{
    this->bDAC.resize(14);
    this->Spare.resize(14);
    for(unsigned int i=0; i<Spare.size(); i++)
    {
        this->Spare[i] = false;
        this->bDAC[i] = 0;
    }
}

globalconfig::~globalconfig()
{
}

void globalconfig::SetbDAC(int i, int value)
{
    this->bDAC[i] = value;
}

void globalconfig::SetSpare(int i, bool value)
{
    this->Spare[i] = value;
}


bool globalconfig::GetSpare(int i)
{
    if(CheckRange(i))
        return this->Spare[i];
    else
        return false;
}

int globalconfig::GetbDAC(int i)
{
    if(CheckRange(i))
        return this->bDAC[i];
    else
        return 0;
}

bool globalconfig::CheckRange(int i)
{
    if(i >= 0 && i < 14)
        return true;
    else
        return false;
}


// ---------------------------------------------------------------------
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CLASS PCBconfig
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ---------------------------------------------------------------------

PCBconfig::PCBconfig()
{
    th1 = 2.;
    th2 = 2.;
    inj = 1.;
}

PCBconfig::~PCBconfig()
{
}

void PCBconfig::SetInj(double voltage)
{
    if(CheckRange(voltage))
        this->inj = voltage;
}

void PCBconfig::SetTh1(double th1)
{
    if(CheckRange(th1))
        this->th1 = th1;
}

void PCBconfig::SetTh2(double th2)
{
    if(CheckRange(th2))
        this->th2 = th2;
}

double PCBconfig::GetInj()
{
    return this->inj;
}

double PCBconfig::GetTh1()
{
    return this->th1;
}

double PCBconfig::GetTh2()
{
    return this->th2;
}

bool PCBconfig::CheckRange(double voltage)
{
    if(voltage >= 0 && voltage < 3.3)
        return true;
    else
        return false;
}
