#ifndef RESULTWRITER_H
#define RESULTWRITER_H

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<time.h>
#include<iomanip>
#include<iostream>
#include<istream>
#include<fstream>
#include<string>
#include<vector>
#include<sys/stat.h>
#include<unistd.h>
#include<stdio.h>
#include"logging.h"

typedef enum
{
  reader=1,
  writer,
} optype;

class boxreadwriter
{
public:
  boxreadwriter(std::string &path,const std::string &seqname,const optype &op)
  {
    filepath_ = path;
    filename_ = path +  seqname;
    op_ = op;


    // if the file exists, remove it and create a new one
    struct stat buffer;
    if(stat (filename_.c_str(), &buffer) == 0)
      {
       std::remove(filename_.c_str());
      }

    file_.open(filename_ , std::ofstream::out | std::ofstream::app);
    if(file_.fail())
      {
        printerror("\rCan't open tabledata");
      }
  }



  ~boxreadwriter()
  {
    file_.close();
  }
  bool writebb(const int &frm, const std::vector<cv::Rect>&rt, const std::vector<int>&tid = std::vector<int>());
private:
  std::string filepath_;
  std::string filename_;
  std::ofstream file_;
  optype op_;
};

#endif // RESULTWRITER_H

