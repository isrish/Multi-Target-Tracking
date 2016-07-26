#include "resultwriter.h"


bool  boxreadwriter::writebb(const int &frm, const std::vector<cv::Rect>&rt,const std::vector<int> &tid)
{
    bool ret = false;
    if(file_.is_open())
    {
        for(size_t i =0; i<rt.size();i++)
        {
            file_<<frm <<", " <<tid[i]<<", "<< rt[i].x <<", " << rt[i].y <<", " << rt[i].width <<", " << rt[i].height <<", "<<0.0<<", "<<-1 <<", "<<-1<<", "<<-1<<"\n";
        }
        file_.flush();
        ret = true;
    }
    else
      {
        printerror("can not write\n");
      }
    return ret;
}


