#ifndef PLOT_H
#define PLOT_H

#include<vector>

class xydata
{
    private:
        std::vector<double> x;
        std::vector<double> y;
    public:
        xydata();
        ~xydata();
        void SetXY(double x, double y);
        double* GetX();
        double* GetY();
        double GetX(int i);
        double GetY(int i);
        int size();
        bool CheckRange(int i);
        void Clear();
};

#endif // PLOT_H
