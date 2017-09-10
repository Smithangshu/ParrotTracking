#ifndef OBJECTMODEL_H
#define OBJECTMODEL_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <vector>

class ObjectModel {
public:
    std::vector<cv::Mat> templates;

    virtual void calculateModel() = 0;
    virtual std::string getTitle() = 0;
};

#endif // OBJECTMODEL_H
