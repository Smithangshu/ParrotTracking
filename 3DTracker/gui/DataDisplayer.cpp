#include "gui/DataDisplayer.h"
#include "gui/MainWindow.h"
#include "gui/DataDisplayerWindow.h"
#include "common/TrackingCommon.h"

DataDisplayer::DataDisplayer()
{
    firstTimeLaunched = true;
}

void DataDisplayer::initDataDisplayerWindow()
{
    if (!TrackingCommon::dataDisplayerWindow->isVisible())
    {
        TrackingCommon::dataDisplayerWindow->show();
    }

    if (firstTimeLaunched)
    {
        firstTimeLaunched = false;
    }
}

void DataDisplayer::displayData(string tabName, Mat &mat, DataDisplayModes dataDisplayMode)
{
    initDataDisplayerWindow();

    // Search for window name
    dataDisplayerIterator = dataDisplayerWindowsInfo.find(tabName);
    if (dataDisplayerIterator == dataDisplayerWindowsInfo.end())
    {
        int tabIndex = TrackingCommon::dataDisplayerWindow->addTab(tabName);
        if (dataDisplayMode == APPEND)
        {
            TrackingCommon::dataDisplayerWindow->setHeaders(tabName, mat.cols * mat.rows);
        }
        else if (dataDisplayMode == FULL)
        {
            TrackingCommon::dataDisplayerWindow->setHeaders(tabName, mat.cols);
        }

        dataDisplayerWindowsInfo[tabName].tabIndex = tabIndex;
    }

    if (dataDisplayMode == APPEND)
    {
        cv::Mat rowData = mat.reshape(0, 1);
        TrackingCommon::dataDisplayerWindow->appendData(tabName, rowData);
    }
    else if (dataDisplayMode == FULL)
    {
        TrackingCommon::dataDisplayerWindow->clearData(tabName);
        for (int i = 0; i < mat.rows; ++i)
        {
            cv::Mat rowData = mat.row(i);
            TrackingCommon::dataDisplayerWindow->appendData(tabName, rowData);
        }
    }
}

void DataDisplayer::displayCoordinates(std::string tabName, std::string displayName, cv::Mat displayData)
{
    initDataDisplayerWindow();
    // Search for window name
    dataDisplayerIterator = dataDisplayerWindowsInfo.find(tabName);
    if (dataDisplayerIterator == dataDisplayerWindowsInfo.end())
    {
        int tabIndex = TrackingCommon::dataDisplayerWindow->addTab(tabName, false);

        dataDisplayerWindowsInfo[tabName].tabIndex = tabIndex;
    }
    dataDisplayerDisplayIterator = dataDisplayerDisplayInfo.find(displayName);
    if (dataDisplayerDisplayIterator == dataDisplayerDisplayInfo.end())
    {
        int rowIndex = TrackingCommon::dataDisplayerWindow->addDisplay(tabName, displayName);
        dataDisplayerDisplayInfo[displayName].rowIndex = rowIndex;
    }
    TrackingCommon::dataDisplayerWindow->setDisplayValues(tabName, displayName, displayData);
}

void DataDisplayer::displayPose(std::string tabName, std::string displayName, cv::Mat displayData)
{
    initDataDisplayerWindow();
    // Search for window name
    dataDisplayerIterator = dataDisplayerWindowsInfo.find(tabName);
    if (dataDisplayerIterator == dataDisplayerWindowsInfo.end())
    {
        int tabIndex = TrackingCommon::dataDisplayerWindow->addTab(tabName, false);

        dataDisplayerWindowsInfo[tabName].tabIndex = tabIndex;
    }
    poseDisplayerDisplayIterator = poseDisplayerDisplayInfo.find(displayName);
    if (poseDisplayerDisplayIterator == poseDisplayerDisplayInfo.end())
    {
        int rowIndex = TrackingCommon::dataDisplayerWindow->addDisplay(tabName, displayName);
        poseDisplayerDisplayInfo[displayName].rowIndex = rowIndex;
    }
    TrackingCommon::dataDisplayerWindow->setPoseValues(tabName, displayName, displayData);
}
