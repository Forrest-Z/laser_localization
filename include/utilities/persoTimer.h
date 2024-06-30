#ifndef LASER_LOCALIZATION_PERSOTIMER_H
#define LASER_LOCALIZATION_PERSOTIMER_H

#include <iostream>
#include <string>
#include <time.h>
#include <ctime>

class persoTimer
{
public:
    persoTimer();
    ~persoTimer();

    void   tic();
    double toc();
    double toc(std::string str);

private:
    clock_t t_;
    double  dt_;
};

#endif //LASER_LOCALIZATION_PERSOTIMER_H
