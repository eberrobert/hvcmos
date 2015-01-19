#include"plot.h"
#include<vector>

xydata::xydata()
{
}

xydata::~xydata()
{

}

bool xydata::CheckRange(int i)
{
    if(i < 0 || i > (int)this->x.size())
        return false;
    else
        return true;
}

void xydata::SetXY(double x, double y)
{
    this->x.push_back(x);
    this->y.push_back(y);
}

double xydata::GetX(int i)
{
    if(CheckRange(i))
        return x[i];
    else
        return 0;
}

double xydata::GetY(int i)
{
    if(CheckRange(i))
        return y[i];
    else
        return 0;
}

double* xydata::GetX()
{
    double* xx = new double[this->x.size()];
    for(unsigned int i=0; i<this->x.size(); i++)
    {
        xx[i] = this->x[i];
    }
    return xx;
}

double* xydata::GetY()
{
    double* xx = new double[this->y.size()];
    for(unsigned int i=0; i<this->y.size(); i++)
    {
        xx[i] = this->y[i];
    }
    return xx;
}

int xydata::size()
{
    return this->x.size();
}

void xydata::Clear()
{
    this->x.clear();
    this->y.clear();
}
