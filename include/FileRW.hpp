#ifndef __FileRW_HPP__
#define __FileRW_HPP__

#include <string>

namespace FileRW
{
    class FileRW
    {
        public:
            FileRW(std::string _path);
            void writef(std::string _string) const;
            void printf(std::string msg) const;
        private:
            std::string path_;
    };
}

#endif

