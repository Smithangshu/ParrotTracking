#ifndef DATADISPLAYER_H
#define DATADISPLAYER_H

#include <QDebug>
#include <QWidget>
#include <opencv2/core/core.hpp>
#include <sstream>
#include "DataDisplayerWindow.h"
#include <map>

class DataDisplayer
{
    private:
        bool firstTimeLaunched;
    public:
        enum DataDisplayModes {APPEND, FULL};
        DataDisplayerWindow* dataDisplayerWindow;
        struct DataDisplayerWindowInfo
        {
            bool created;
            int cols;
            int rows;
            int tabIndex;
        };
        struct DataDisplayerDisplayInfo
        {
            bool created;
            int rowIndex;
        };
        struct PoseDisplayerDisplayInfo
        {
            bool created;
            int rowIndex;
        };
        std::map<std::string, DataDisplayerWindowInfo> dataDisplayerWindowsInfo;
        std::map<std::string, DataDisplayerWindowInfo>::iterator dataDisplayerIterator;

        std::map<std::string, DataDisplayerDisplayInfo> dataDisplayerDisplayInfo;
        std::map<std::string, DataDisplayerDisplayInfo>::iterator dataDisplayerDisplayIterator;

        std::map<std::string, PoseDisplayerDisplayInfo> poseDisplayerDisplayInfo;
        std::map<std::string, PoseDisplayerDisplayInfo>::iterator poseDisplayerDisplayIterator;

        DataDisplayer();
        void initDataDisplayerWindow();
        void displayData(std::string tabName, cv::Mat &mat, enum DataDisplayModes dataDisplayMode = APPEND);
        void displayCoordinates(std::string tabName, std::string displayName, cv::Mat displayData);
        void displayPose(std::string tabName, std::string displayName, cv::Mat displayData);
};

#endif // DATADISPLAYER_H
