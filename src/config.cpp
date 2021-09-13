#include "config.h"

namespace myslam
{
    bool Config::SetParameterFile(const std::string &filename)
    {
        if(config_ == nullptr)//单例类只创建一次
        {
            config_ = std::shared_ptr<Config>(new Config);
            config_->file_ = cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        }
        if(config_->file_.isOpened())
        {
            return true;
        } else
        {
            config_->file_.release();//如果没读到任何东西，就把这部分内存释放掉。
            return false;
        }
    }
    std::shared_ptr<Config> Config::config_ = nullptr;//为满足单例模式编程，先初始化为空指针
}