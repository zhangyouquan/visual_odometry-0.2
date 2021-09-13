#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

namespace myslam
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config(){}//构造函数私有化(单例类)

    public:
        ~Config()
        {
            if( file_.isOpened())
            {
                file_.release();
            }
        };

        static bool SetParameterFile(const std::string &filename);

        //取配置文件中的数据，因为配置文件内容类型可能不同，这里用模板类编程
        template<class T>
        static T Get(const std::string &key)
        {
                 return T(Config::config_->file_[key]);
        }

    };
}
#endif
