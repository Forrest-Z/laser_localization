#include "utilities/persoTimer.h"

persoTimer::persoTimer() : t_(), dt_(0.0)
{
}


persoTimer::~persoTimer()
{
}

void persoTimer::tic()
{
    t_ = clock();

}

double persoTimer::toc()
{
    dt_ = (float) (clock() - t_) / ((float) CLOCKS_PER_SEC);
    std::cout << "Elapsed time : " << dt_ << std::endl;

    return dt_;
}

double persoTimer::toc(std::string str)
{
    dt_ = (float)(clock() - t_) / ((float)CLOCKS_PER_SEC);
    std::cout << "Elapsed time (" << str << ") : " << dt_ << std::endl;
    return dt_;
}
