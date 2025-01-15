#include "FileRW.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>

using namespace std;

namespace FileRW
{
    FileRW::FileRW(string _path)
    {
        this->path_ = _path;
        filesystem::path dir = filesystem::path(_path).parent_path();
        if(!filesystem::exists(dir))
        {
            filesystem::create_directories(dir);
        }
    }

    void FileRW::printf(string msg) const
    {
        cout << msg << endl;
    }

    void FileRW::writef(string _string) const
    {
        fstream file;
        file.open(this->path_, std::ios_base::app);
        if(file.is_open())
        {
            string text = _string;
            //! this->printf(this->path_ + " : " + text);
            file.write(text.data(), text.size());
        }
    }
}

