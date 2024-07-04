#ifndef LASER_LOCALIZATION_PLYFILE_H
#define LASER_LOCALIZATION_PLYFILE_H

#include <sstream>
#include <list>
#include "file.h"

enum plyFormat{ binary_little_endian = 0, binary_big_endian = 1, ascii = 2 };
enum plyTypes{ float32 = 0, float64 = 1, uchar = 2, int32 = 3, otherxx = -1 };

class plyFile : public file
{
public:
    plyFile(std::string path, openMode flag);
    ~plyFile();

    void readFile(char*& points, int& pointSize, int& numPoints);
    void writeFile(char* points, int numPoints, std::list<std::string> properties, std::list<plyTypes> types);

    void displayInfos();

    static const int READ_SIZE = 16000;
    static const int WRITE_SIZE = 16000;

private:
    void readHeader();
    void writeHeader();


private:
    std::string    header_;
    plyFormat format_;

    int       propertyNum_;
    std::string*   propertyName_;
    plyTypes* propertyType_;
    int*	  propertySize_;

    int   numPoints_;
    int   pointSize_;
};



#endif //LASER_LOCALIZATION_PLYFILE_H
