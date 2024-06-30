#include "utilities/plyFile.h"

plyFile::plyFile(std::string path, openMode flag) :
file(path, flag), header_(""), format_(binary_little_endian),
propertyNum_(0), propertyType_(nullptr), propertySize_(nullptr),
propertyName_(nullptr), numPoints_(0), pointSize_(0)
{
    if (mode_ == fileOpenMode_IN)
    {
        readHeader();
    }
}


plyFile::~plyFile()
{
    delete[] propertyType_;
    delete[] propertySize_;
    delete[] propertyName_;
}


void plyFile::readHeader()
{
    // GET HEADER
    // ------------------------------------------------------------------------------------------
    std::string tmpStr = "";

    do
    {
        getline(file_, tmpStr);
        header_ += tmpStr + "\n";
    } while (tmpStr.find("endheader_") != 0);


    // PARSE HEADER
    // ------------------------------------------------------------------------------------------
    std::stringstream streamHeader(header_);
    std::string strTmp = "";
    std::list<plyTypes> typePptTmp;
    std::list<int>      sizePptTmp;
    std::list<std::string>   namePptTmp;

    while (!streamHeader.eof())
    {
        streamHeader >> strTmp;

        if (strTmp.compare("format") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("binary_little_endian") == 0)      format_ = binary_little_endian;
            else if (strTmp.compare("binary_big_endian") == 0)    format_ = binary_big_endian;
            else if (strTmp.compare("ascii") == 0)                format_ = ascii;
        }

        if (strTmp.compare("element") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("vertex") == 0) streamHeader >> numPoints_;
        }

        if (strTmp.compare("property") == 0)
        {
            propertyNum_++;
            streamHeader >> strTmp;
            if ((strTmp.compare("float32") == 0) | (strTmp.compare("float") == 0))
            {
                typePptTmp.push_back(float32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("float64") == 0) | (strTmp.compare("double") == 0))
            {
                typePptTmp.push_back(float64);
                sizePptTmp.push_back(8);
            }
            else if ((strTmp.compare("int") == 0))
            {
                typePptTmp.push_back(int32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("uchar") == 0))
            {
                typePptTmp.push_back(uchar);
                sizePptTmp.push_back(1);
            }
            else
            {
                typePptTmp.push_back(otherxx);
                sizePptTmp.push_back(4); // Default
            }

            streamHeader >> strTmp;
            namePptTmp.push_back(strTmp);
        }
    }


    // FILL PROPERTIES ARRAYS
    // ------------------------------------------------------------------------------------------
    propertyType_ = new plyTypes[propertyNum_];
    propertySize_ = new int[propertyNum_];
    propertyName_ = new std::string[propertyNum_];

    for (int i(0); i < propertyNum_; i++)
    {
        propertyType_[i] = typePptTmp.front();
        propertySize_[i] = sizePptTmp.front();
        propertyName_[i] = namePptTmp.front();
        typePptTmp.pop_front();
        sizePptTmp.pop_front();
        namePptTmp.pop_front();

        pointSize_ += propertySize_[i];
    }
}


void plyFile::writeHeader()
{
    header_ = "";


    header_ += "ply";
    header_ += "\n";


    header_ += "format ";
    switch (format_)
    {
        case binary_little_endian:
        {
            header_ += "binary_little_endian 1.0";
            break;
        }
        case binary_big_endian:
        {
            header_ += "binary_big_endian 1.0";
            break;
        }
        case ascii:
        {
            header_ += "binary_ascii 1.0";
            break;
        }
    }
    header_ += "\n";


    header_ += "element vertex ";
    header_ += std::to_string(numPoints_);
    header_ += "\n";


    for (int i(0); i < propertyNum_; i++)
    {
        header_ += "property ";
        switch (propertyType_[i])
        {
            case float32:
            {
                header_ += "float32 ";
                break;
            }
            case float64:
            {
                header_ += "float64 ";
                break;
            }
            case uchar:
            {
                header_ += "uchar ";
                break;
            }
            case int32:
            {
                header_ += "int ";
                break;
            }
        }
        header_ += propertyName_[i];
        header_ += "\n";
    }


    header_ += "endheader_";
    header_ += "\n";

    file_ << header_;
}



void plyFile::readFile(char*& points, int& pointSize, int& numPoints)
{
    switch (format_)
    {
        case binary_little_endian:
        {
            // ----- Allocate memory ------------------------------------------------
            if (points != 0)
            {
                delete[] points;
            }

            points = new char[(unsigned long long int)pointSize_*(unsigned long long int)numPoints_];
            unsigned long long int bufferSize = (unsigned long long int)pointSize_*(unsigned long long int)numPoints_;


            // ----- Read raw data --------------------------------------------------
            unsigned long long int n = bufferSize / (unsigned long long int)READ_SIZE;
            unsigned long long int r = bufferSize % (unsigned long long int)READ_SIZE;

            for (unsigned long long int i(0); i < n; i++)
            {
                file_.read(points + i*(unsigned long long int)READ_SIZE, READ_SIZE);
            }

            file_.read(points + n*(unsigned long long int)READ_SIZE, r);

            numPoints = numPoints_;
            pointSize = pointSize_;

            break;
        }
        case binary_big_endian:
        {
            std::cout << "WARNING: function not implemented for binary big endian file" << std::endl;
            break;
        }
        case ascii:
        {
            std::cout << "WARNING: function not implemented for ascii file" << std::endl;
            break;
        }
    }
}



void plyFile::writeFile(char* points, int numPoints, std::list<std::string> properties, std::list<plyTypes> types)
{
    // ----- Set properties -------------------------------------------------
    format_ = binary_little_endian;
    numPoints_ = numPoints;

    if (properties.size() != types.size())
    {
        std::cout << "Warning : mismatch between properties and types" << std::endl;
        return;
    }

    propertyNum_ = (int) properties.size();
    pointSize_ = 0;

    propertyType_ = new plyTypes[propertyNum_];
    propertySize_ = new int[propertyNum_];
    propertyName_ = new std::string[propertyNum_];


    auto propIt = properties.begin();
    auto typesIt = types.begin();
    int i = 0;

    for (int i(0); i < propertyNum_; i++)
    {
        propertyName_[i] = *propIt;
        propertyType_[i] = *typesIt;


        if (propertyType_[i] == float32)
            propertySize_[i] = 4;
        if (propertyType_[i] == float64)
            propertySize_[i] = 8;
        if (propertyType_[i] == int32)
            propertySize_[i] = 4;
        if (propertyType_[i] == uchar)
            propertySize_[i] = 1;
        if (propertyType_[i] == otherxx)
            propertySize_[i] = 4; //default

        pointSize_ += propertySize_[i];

        ++propIt;
        ++typesIt;
    }


    // ----- Write header ---------------------------------------------------
    writeHeader();


    // ----- Write points ---------------------------------------------------
    unsigned long long int bufferSize = (unsigned long long int)pointSize_*(unsigned long long int)numPoints_;

    unsigned long long int n = bufferSize / (unsigned long long int)WRITE_SIZE;
    unsigned long long int r = bufferSize % (unsigned long long int)WRITE_SIZE;

    for (unsigned long long int i(0); i < n; i++)
    {
        file_.write(points + i*(unsigned long long int)WRITE_SIZE, WRITE_SIZE);
    }

    file_.write(points + n*(unsigned long long int)WRITE_SIZE, r);
}



void plyFile::displayInfos()
{
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << " PLY File : " << path_ << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "  - format     : " << format_ << std::endl;
    std::cout << "  - num points : " << numPoints_ << std::endl;
    std::cout << "  - properties : " << std::endl;
    for (int i(0); i < propertyNum_; i++)
    {
        std::cout << "     - " << propertyName_[i] << " :	" << propertyType_[i] << " |	" << propertySize_[i] << " bytes " << std::endl;
    }
    std::cout << "------------------------------------------------------" << std::endl << std::endl;
}