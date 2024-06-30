#include "utilities/file.h"

file::file(std::string path, openMode flag) : path_(path), mode_(flag)
{
    switch (mode_)
    {
        case fileOpenMode_IN:
        {
            file_ = std::fstream(path_, std::ios::in | std::ios::binary);
            break;
        }
        case fileOpenMode_OUT:
        {
            file_ = std::fstream(path_, std::ios::out | std::ios::binary);
            break;
        }
    }

    if (!file_.good())
    {
        std::cout << "ERROR: can't open " << path_ << std::endl;
    }
}


file::~file()
{
    file_.close();
}

