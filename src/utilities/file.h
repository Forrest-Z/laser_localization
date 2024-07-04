#ifndef LASER_LOCALIZATION_FILE_H
#define LASER_LOCALIZATION_FILE_H

#include <iostream>
#include <string>
#include <fstream>


enum openMode{ fileOpenMode_OUT = 0, fileOpenMode_IN = 1 };

class file
{
public:
    file(std::string path, openMode flag);
    ~file();

protected:
    const openMode mode_;
    std::string     path_;
    std::fstream    file_;
};


#endif //LASER_LOCALIZATION_FILE_H
