/**
* Software License Agreement (BSD License)
*
*           Copyright (C) 2015 by Israel D. Gebru,
*           Perception Team, INRIA-Grenoble, France
*                   All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*/

#ifndef LOGGING_H
#define LOGGING_H

#include "stdio.h"
#include "stdlib.h"
#include "iostream"
#include "string.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define ANSI_COLOR_REDB    "\x1b[41m"
#define ANSI_COLOR_GREENB  "\x1b[42m"
#define ANSI_COLOR_RESETB  "\x1b[49m"

inline void printerror(std::string str)
{
    printf(ANSI_COLOR_RED"%s"ANSI_COLOR_RESET"\n",str.c_str());
}

inline void printinfo(std::string str)
{
    printf(ANSI_COLOR_GREEN"%s"ANSI_COLOR_RESET"\n",str.c_str());
}

inline void printwarning(std::string str)
{
    printf(ANSI_COLOR_YELLOW"%s"ANSI_COLOR_RESET"\n",str.c_str());
}


inline void printother(std::string str)
{
    printf(ANSI_COLOR_BLUE "%s"ANSI_COLOR_RESET"\n",str.c_str());
}

inline void printotherBggreen(std::string str)
{
   printf(ANSI_COLOR_GREENB"%s"ANSI_COLOR_RESETB"\n",str.c_str());
}

inline void printotherBgred(std::string str)
{
   printf(ANSI_COLOR_REDB"%s"ANSI_COLOR_RESETB"\n",str.c_str());
}

#endif // LOGGING_H

