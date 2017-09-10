#ifndef FEATUREBASEDMODEL_H
#define FEATUREBASEDMODEL_H

#include "ObjectModel.h"

class FeatureBasedModel : public ObjectModel {
public:
    cv::Mat homography;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    void calculateModel();
    std::string getTitle();
};

#endif // FEATUREBASEDMODEL_H
