#include "FeatureBasedModel.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <sstream>
#include <QDebug>

void FeatureBasedModel::calculateModel()
{
    if (templates.size() > 0)
    {
        cv::Mat image = templates.at(0);
        cv::OrbFeatureDetector detector;
        detector.detect(image, keypoints);

        cv::OrbDescriptorExtractor extractor;
        extractor.compute(image, keypoints, descriptors);
    }
    else
    {
        qDebug() << "Trying to find model, but there is no image yet";
    }
}

std::string FeatureBasedModel::getTitle()
{
    std::stringstream title;
    title << "FeatureBased " << keypoints.size();
    return title.str();
}
