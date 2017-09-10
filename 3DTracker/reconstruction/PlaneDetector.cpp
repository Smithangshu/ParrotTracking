#include "PlaneDetector.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common/TrackingCommon.h"
#include <QDebug>

PlaneDetector::PlaneDetector()
{
    faces = 0;
    frameSkip =  0;
    planeDetectionMethod = FLOW_HOMOGRAPHY;
    modelFound = false;
}

void PlaneDetector::addImage(cv::Mat image)
{
    images.push_back(image.clone());
    if (images.size() > 4)
        images.pop_front();
}

void PlaneDetector::setMethod(PlaneDetectionMethods planeDetectionMethod)
{
    this->planeDetectionMethod = planeDetectionMethod;
}

bool PlaneDetector::findPlanes(cv::Mat &dst)
{
    switch (planeDetectionMethod)
    {
        case FLOW_HOMOGRAPHY: return findPlanesFlowHomography(dst);
        case FEATURE_MATCHING: return findPlanesFeatures(dst);
        case BINARY_SEARCH: return findPlanesBinary(dst);
        case BINARY_SEARCH_DISTINCT: return findPlanesBinaryDistinctColors(dst);
    }
    return false;
}

bool PlaneDetector::findPlanesBackProjection(cv::Mat &dst)
{
   return false;
}

bool PlaneDetector::findPlanesFlowHomography(cv::Mat &dst)
{
    cv::Mat flwX, flwY;
    if (!estimateDenseFlowCPU(flwX, flwY))
        return false;
    cv::Mat altDst = dst.clone();

    std::vector<int> xSampleCoordinates;
    std::vector<int> ySampleCoordinates;
    std::vector<int> xSampleFlow;
    std::vector<int> ySampleFlow;
    std::vector<cv::Point2f> sourcePoints;
    std::vector<cv::Point2f> targetPoints;

    int rows = dst.rows;
    int cols = dst.cols;
    uchar* referencePointerX;
    uchar* referencePointerY;
    for (int y = 0; y < rows; ++y)
    {
        referencePointerX = flwX.ptr(y);
        referencePointerY = flwY.ptr(y);
        for (int x = 0; x < cols; ++x)
        {
            float dx = *((float*) (referencePointerX + 4 * x));
            float dy = *((float*) (referencePointerY + 4 * x));
            bool acceptLine = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 4.0;
            if (acceptLine)
            {
                xSampleCoordinates.push_back(x);
                ySampleCoordinates.push_back(y);
                xSampleFlow.push_back(dx);
                ySampleFlow.push_back(dy);
                sourcePoints.push_back(cv::Point2f(x, y));
                targetPoints.push_back(cv::Point2f((float)x + dx, (float)y + dy));
                //qDebug() << "Drawing line";
                //line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
                //line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
            }
            if (x % 10 == 0 && y % 10 == 0 && acceptLine) {
                line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
                line(altDst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
            }
        }
    }
    imshow("altDst", altDst);
    images.clear();

    if (xSampleCoordinates.size() > 20)
    {
        std::vector<cv::Point2f> srcFullPoints;
        std::vector<cv::Point2f> dstFullPoints;
        std::vector<cv::Point2f> srcPoints;
        std::vector<cv::Point2f> dstPoints;
        std::vector<cv::Point2f> dstHomographyPoints;
        for (int i = 0; i < xSampleCoordinates.size(); ++i)
        {
            srcFullPoints.push_back(cv::Point2f((float) xSampleCoordinates.at(i), (float) ySampleCoordinates.at(i)));
            dstFullPoints.push_back(cv::Point2f((float) xSampleCoordinates.at(i) + (float) xSampleFlow.at(i), (float) ySampleCoordinates.at(i) + (float) ySampleFlow.at(i)));
            if (std::sqrt(std::pow((float) xSampleFlow.at(i), 2) + std::pow((float) ySampleFlow.at(i), 2)) > 30.0) {
                srcPoints.push_back(cv::Point2f((float) xSampleCoordinates.at(i), (float) ySampleCoordinates.at(i)));
                dstPoints.push_back(cv::Point2f((float) xSampleCoordinates.at(i) + (float) xSampleFlow.at(i), (float) ySampleCoordinates.at(i) + (float) ySampleFlow.at(i)));
            }
        }
        cv::Mat homography = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 2);

        perspectiveTransform(srcFullPoints, dstHomographyPoints, homography);
        for (int i = 0; i < srcFullPoints.size(); ++i)
        {
            cv::Point2f point = dstFullPoints.at(i);
            cv::Point2f pointHomography = dstHomographyPoints.at(i);
            if (std::sqrt(std::pow(pointHomography.x - point.x, 2) + std::pow(pointHomography.y - point.y, 2)) < 15.0) {
                cv::circle(dst, point, 2, cv::Scalar(0, 255, 0));
            }
        }
    }
    return false;
}

bool PlaneDetector::findPlanesFlowHomographyCorrected(cv::Mat &dst)
{
    using namespace cv;

    cv::Mat flwX, flwY;
    if (!estimateDenseFlowCPU(flwX, flwY))
        return false;

    std::vector<int> xSampleCoordinates;
    std::vector<int> ySampleCoordinates;
    std::vector<int> xSampleFlow;
    std::vector<int> ySampleFlow;
    std::vector<cv::Point2f> sourcePoints;
    std::vector<cv::Point2f> targetPoints;

    int rows = dst.rows;
    int cols = dst.cols;
    uchar* referencePointerX;
    uchar* referencePointerY;
    for (int y = 0; y < rows; ++y)
    {
        referencePointerX = flwX.ptr(y);
        referencePointerY = flwY.ptr(y);
        for (int x = 0; x < cols; ++x)
        {
            float dx = *((float*) (referencePointerX + 4 * x));
            float dy = *((float*) (referencePointerY + 4 * x));
            if (dx*dx > 8 || dy*dy > 8)
            {
                xSampleCoordinates.push_back(x);
                ySampleCoordinates.push_back(y);
                xSampleFlow.push_back(dx);
                ySampleFlow.push_back(dy);
                sourcePoints.push_back(cv::Point2f(x, y));
                targetPoints.push_back(cv::Point2f((float)x + dx, (float)y + dy));
                //qDebug() << "Drawing line";
                //line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
                line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
            }
            //line(dst, Point(x, y), Point(x + dx, y + dy), Scalar(0, 0, 255));
        }
    }
    images.clear();

    if (xSampleCoordinates.size() > 20)
    {
        cv::Mat samples = cv::Mat(xSampleCoordinates.size(), 1, CV_32FC4);
        cv::Mat labels;
        Mat centers(3, 1, CV_32FC4);
        for (int i = 0; i < xSampleCoordinates.size(); ++i)
        {
            samples.at<Vec4f>(i, 0)[0] = (float) xSampleCoordinates.at(i);
            samples.at<Vec4f>(i, 0)[1] = (float) ySampleCoordinates.at(i);
            samples.at<Vec4f>(i, 0)[2] = (float) xSampleFlow.at(i);
            samples.at<Vec4f>(i, 0)[3] = (float) ySampleFlow.at(i);
        }
        cv::kmeans(samples, 3, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0), 3, cv::KMEANS_RANDOM_CENTERS, centers);
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        int totalsPerGroup[3];
        int currentGroupIndex[3];
        for (int i = 0; i < 3; ++i)
        {
            totalsPerGroup[i] = 0;
            currentGroupIndex[i] = 0;
        }

        for (int i = 0; i < labels.rows; ++i)
        {
            int labelIndex = labels.at<int>(i, 0);
            qDebug() << "labelIndex " << labelIndex;
            cv::Point point;
            point.x = (float) xSampleCoordinates.at(i);
            point.y = (float) ySampleCoordinates.at(i);
            cv::circle(dst, point, 1, colors[labelIndex]);
            ++totalsPerGroup[labelIndex];
        }
        //return false;

        // Now, find an homography for each cluster
        // Should be integrated with previous step
        cv::Mat sourcePlanePoints[3];
        cv::Mat targetPlanePoints[3];
        cv::Mat homographies[3];
        for (int i = 0; i < 3; ++i)
        {
            // Validate minimum points for homography
            if (totalsPerGroup[i] < 4)
                return false;
            sourcePlanePoints[i] = cv::Mat(totalsPerGroup[i], 2, CV_32F);
            targetPlanePoints[i] = cv::Mat(totalsPerGroup[i], 2, CV_32F);
        }
        for (int i = 0; i < samples.rows; ++i)
        {
            int labelIndex = labels.at<int>(i, 0);
            int index = currentGroupIndex[labelIndex];

            sourcePlanePoints[labelIndex].at<float>(index, 0) = sourcePoints.at(i).x;
            sourcePlanePoints[labelIndex].at<float>(index, 1) = sourcePoints.at(i).y;
            targetPlanePoints[labelIndex].at<float>(index, 0) = targetPoints.at(i).x;
            targetPlanePoints[labelIndex].at<float>(index, 1) = targetPoints.at(i).y;

            ++currentGroupIndex[labelIndex];
        }

//        for (int i = 0; i < 3; ++i)
//        {
//            for (int j = 0; j < sourcePlanePoints[i].rows; ++j)
//                qDebug() << "Mat " << i << " PointS: " << sourcePlanePoints[i].at<float>(j, 0) << ", " << sourcePlanePoints[i].at<float>(j, 1) << " PointE: " << targetPlanePoints[i].at<float>(j, 0) << ", " << targetPlanePoints[i].at<float>(j, 1);
//        }

        for (int i = 0; i < 3; ++i)
        {
            homographies[i] = cv::findHomography(sourcePlanePoints[i], targetPlanePoints[i], cv::RANSAC);
            homographies[i].convertTo(homographies[i], CV_32F);
        }

        // Now, compare each point to the each homography. Align point to the best homography
        cv::Mat samplePoint = cv::Mat(3, 1, CV_32F);
        samplePoint.setTo(1.0);
        cv::Mat newPoint = cv::Mat(3, 1, CV_32F);
        float newPointLength;
        cv::Mat tempPoint = cv::Mat(3, 1, CV_32F);
        float tempPointLength;
        cv::Mat newLabels = labels.clone();

        // For displaying
        cv::Mat correctedGroups; // = srcNxt.clone();
        for (int i = 0; i < samples.rows; ++i)
        {
            samplePoint.at<float>(0, 0) = sourcePoints.at(i).x;
            samplePoint.at<float>(1, 0) = sourcePoints.at(i).y;
            newPoint = homographies[0] * samplePoint;
            newPoint /= newPoint.at<float>(2, 0);
            newPointLength = std::sqrt(std::pow(newPoint.at<float>(0, 0), 2) + std::pow(newPoint.at<float>(1, 0), 2));
            newLabels.at<int>(i, 0) = 0;

            for (int j = 1; j < 3; ++j)
            {
                tempPoint = homographies[j] * samplePoint;
                tempPoint /= tempPoint.at<float>(2, 0);
                tempPointLength = std::sqrt(std::pow(tempPoint.at<float>(0, 0), 2) + std::pow(tempPoint.at<float>(1, 0), 2));
                if (tempPointLength < newPointLength)
                {
                    newPointLength = tempPointLength;
                    newLabels.at<int>(i, 0) = j;
                }
            }
            cv::circle(correctedGroups, cv::Point((int)targetPoints.at(i).x, (int)targetPoints.at(i).y), 1, colors[newLabels.at<int>(i, 0)]);
        }
        cv::imshow("Corrected", correctedGroups);
        cv::waitKey(0);
    }
}

bool PlaneDetector::findPlanesFeatures(cv::Mat &dst)
{

    cv::Mat srcNxt = images[images.size() - 1];

    // Get features from current image
    std::vector<cv::KeyPoint> keypoints;

    //cv::SurfFeatureDetector detector(1000);
    cv::OrbFeatureDetector detector;
    detector.detect(srcNxt, keypoints);

    cv::Mat flwX, flwY;

    if (TrackingCommon::enableMotionFilter)
    {
        if (!estimateDenseFlowGPU(flwX, flwY))
            return false;

        std::vector<cv::KeyPoint> keypointsFiltered;
        if (!filterKeypointsWithDenseFlow(keypoints, keypointsFiltered, flwX, flwY, dst))
        {
            return false;
        }
        keypoints.clear();
        keypoints = keypointsFiltered;
    }

    cv::Mat descriptors;
    cv::OrbDescriptorExtractor extractor;
    extractor.compute(srcNxt, keypoints, descriptors);

    // 5, Find the matches
    for (int i = 0; i < TrackingCommon::objectModels.size(); ++i)
    {
        FeatureBasedModel* model = (FeatureBasedModel*)TrackingCommon::objectModels.at(i);
        std::vector<cv::DMatch> matches;

        //cv::FlannBasedMatcher matcher;
        cv::BFMatcher matcher(cv::NORM_L1);
        matcher.match(model->descriptors, descriptors, matches);

        // For displaying
        std::vector<cv::KeyPoint> matchedKeypoints;
        // For the homography
        std::vector<cv::Point2f> srcPoints;
        std::vector<cv::Point2f> dstPoints;
        for (int j = 0; j < model->descriptors.rows; ++j)
        {
            cv::DMatch dMatch = (DMatch) matches.at(j);
            //qDebug() << "dMatch.distance  " << dMatch.distance ;
            if (dMatch.distance > 1400)
            {
                matchedKeypoints.push_back(keypoints.at(dMatch.trainIdx));
                srcPoints.push_back(((cv::KeyPoint)model->keypoints.at(dMatch.queryIdx)).pt);
                dstPoints.push_back(((cv::KeyPoint)keypoints.at(dMatch.trainIdx)).pt);
                qDebug() << "Training with " << ((cv::KeyPoint)model->keypoints.at(dMatch.queryIdx)).pt.x << "," << ((cv::KeyPoint)model->keypoints.at(dMatch.queryIdx)).pt.y << " - " << ((cv::KeyPoint)keypoints.at(dMatch.trainIdx)).pt.x << "," << ((cv::KeyPoint)keypoints.at(dMatch.trainIdx)).pt.y;
            }
        }
        TrackingCommon::highlightKeypoints(TrackingCommon::currentFrame, matchedKeypoints);

        if (srcPoints.size() >= 4)
        {
            // Find the homography
            cv::Mat originalTemplate = (cv::Mat)model->templates.at(0);
            cv::Mat homography = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 3);
            std::vector<cv::Point2f> srcCorners(4);
            std::vector<cv::Point2f> dstCorners(4);
            srcCorners[0] = cv::Point2f(0, 0);
            srcCorners[1] = cv::Point2f(originalTemplate.cols, 0);
            srcCorners[2] = cv::Point2f(originalTemplate.cols, originalTemplate.rows);
            srcCorners[3] = cv::Point2f(0, originalTemplate.rows);
            cv::perspectiveTransform(srcCorners, dstCorners, homography);
            for (int j = 0; j < 4; ++j)
            {
                qDebug() << "Corner " << j << ": " << ((cv::Point2f)srcCorners[j]).x << "," << ((cv::Point2f)srcCorners[j]).y << " " << ((cv::Point2f)dstCorners[j]).x << "," << ((cv::Point2f)dstCorners[j]).y;
                cv::line(TrackingCommon::currentFrame, dstCorners[j], dstCorners[(j + 1) % 4], cv::Scalar(0, 255, 0), 3);
            }

        }
        else
        {
            qDebug() << "Less than 4 matches were found";
        }
        cv::Mat img_matches;
        cv::drawMatches((cv::Mat)model->templates.at(0), model->keypoints, TrackingCommon::frame, keypoints, matches, img_matches);
        //std::vector<DMatch> test;
        //test.push_back(matches[(rand() % (matches.size() - 1))]);
        //cv::drawMatches((cv::Mat)model->templates.at(0), model->keypoints, TrackingCommon::frame, keypoints, test, img_matches);
        cv::imshow( "Good Matches", img_matches );
        //sleep(5);
        //TrackingCommon::findModels = false;
    }

    //images.clear();
    return true;
}

bool PlaneDetector::findPlanesBinary(cv::Mat &dst)
{
    // First, color binary image
    cv::Mat HSVImage;
    std::vector<cv::Mat> HSVChannels;
    TrackingCommon::frame.convertTo(HSVImage, CV_BGR2HSV);
    cv::split(HSVImage, HSVChannels);
    // Line will be fully saturated
    cv::Mat binaryImage = HSVChannels[1];

    cv::threshold(binaryImage, binaryImage, TrackingCommon::colorHSVThreshold, 250, cv::THRESH_BINARY);

    // Apply canny edge detector
    //cv::Canny(binaryImage, binaryImage, 30, 100);
    //cv::Mat kernel = cv::Mat(5, 5, CV_32F, cv::Scalar(1));
    //cv::dilate(binaryImage, binaryImage, kernel);

    //////////////////////////////////////////////
    // find lines with Hough transform
    //////////////////////////////////////////////

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 40, 20);

    if (lines.size() < 4)
    {
        qDebug() << "No lines were found";
        return false;
    }

    // Get two clusters from data
    // Cluster line segments by using its lengths, on groups of 2
    cv::Mat samples = cv::Mat(lines.size(), 1, CV_32FC4);
    cv::Mat labels;
    cv::Mat centers(2, 1, CV_32FC4);

    // Conver to adequate format for kmeans
    for (int i = 0; i < lines.size(); ++i)
        for (int j = 0; j < 4; ++j)
            samples.at<cv::Vec4f>(i, 0)[j] = lines[i][j];

    cv::kmeans(samples, 2, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);
    cv::Scalar colors[4];
    colors[0] = cv::Scalar(0, 0, 255);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(255, 0, 0);
    colors[3] = cv::Scalar(128, 128, 0);

    //////////////////////////////////////////////
    // find the longest lines for each group
    //////////////////////////////////////////////

    std::vector<double> lineLengths;
    int largestLines[2];
    largestLines[0] = largestLines[1] = 0;
    double length[2];
    length[0] = length[1] = 0.0;
    for (int i = 0; i < labels.rows; ++i)
    {
        int labelIndex = labels.at<int>(i, 0);
        double lineLength = std::sqrt((double)std::pow((double)lines[i][0] - lines[i][2], 2) + std::pow((double)lines[i][1] - lines[i][3], 2));
        if (lineLength > length[labelIndex]) {
            length[labelIndex] = lineLength;
            largestLines[labelIndex] = i;
        }
        cv::Point point;
        point.x = lines[i][0];
        point.y = lines[i][1];
        //cv::circle(dst, point, 3, colors[labelIndex]);
        lineLengths.push_back(lineLength);
    }

    //////////////////////////////////////////////
    // Use vanishing point to determine best solution
    //////////////////////////////////////////////

    // Lets find the to closest sides to the vanishing point
    // First, the intersection point
    double xVanishing, yVanishing;
    cv::Point2f vanishingPoint = calculateVanishingPoint(lines, largestLines);
    if (vanishingPoint.x == 0.0 && vanishingPoint.y == 0.0)
        return false;
    xVanishing = vanishingPoint.x;
    yVanishing = vanishingPoint.y;

    //////////////////////////////////////////////
    // Get all points close to vanishing point
    //////////////////////////////////////////////

    int totalsPerGroup[2];
    totalsPerGroup[0] = totalsPerGroup[1] = 0;
    cv::Point avgCorners[2][2];
    for (int i = 0; i < lines.size(); ++i)
    {
        int labelIndex = labels.at<int>(i, 0);
        double vanishingDistance1 = std::sqrt(std::pow(lines[i][0] - xVanishing, 2) + std::pow(lines[i][1] - yVanishing, 2));
        double vanishingDistance2 = std::sqrt(std::pow(lines[i][2] - xVanishing, 2) + std::pow(lines[i][3] - yVanishing, 2));
        if (vanishingDistance1 < vanishingDistance2)
        {
            // swap points
            cv::Vec4i temp = lines[i];
            lines[i][0] = temp[2];
            lines[i][1] = temp[3];
            lines[i][2] = temp[0];
            lines[i][3] = temp[1];
        }
        // Only consider lines which are large
        if (lineLengths[i] > length[labelIndex] * 0.75)
        {
            avgCorners[labelIndex][0].x += lines[i][0];
            avgCorners[labelIndex][0].y += lines[i][1];
            avgCorners[labelIndex][1].x += lines[i][2];
            avgCorners[labelIndex][1].y += lines[i][3];
            totalsPerGroup[labelIndex]++;
        }
    }

    for (int i = 0; i < 2; ++i) // Each label
    {
        for (int j = 0; j < 2; ++j) // Each point
        {
            avgCorners[i][j].x /= totalsPerGroup[i];
            avgCorners[i][j].y /= totalsPerGroup[i];
            //cv::circle(dst, avgCorners[i][j], 5, cv::Scalar(0, 200, 128), 3);
        }
    }

    for (int i = 0; i < 2; ++i)
    {
        lines[largestLines[i]][0] = avgCorners[i][0].x;
        lines[largestLines[i]][1] = avgCorners[i][0].y;
        lines[largestLines[i]][2] = avgCorners[i][1].x;
        lines[largestLines[i]][3] = avgCorners[i][1].y;
    }

    // Update vanishing point
    vanishingPoint = calculateVanishingPoint(lines, largestLines);
    if (vanishingPoint.x == 0.0 && vanishingPoint.y == 0.0)
        return false;
    xVanishing = vanishingPoint.x;
    yVanishing = vanishingPoint.y;


    //////////////////////////////////////////////
    // Finally, use vanishing line to determine which is left and right side
    //////////////////////////////////////////////

    cv::Point2f middle = cv::Point2f((linePoints[0].x + linePoints[1].x) / 2.0, (linePoints[0].y + linePoints[1].y) / 2.0);
    cv::circle(dst, middle, 5, colors[3], 2);
    // Remove offset of vanishing point
    middle.x = middle.x - xVanishing;
    middle.y = middle.y - yVanishing;
    if (xVanishing != 0)
    {
        double angle = std::atan2(yVanishing, xVanishing);
        qDebug() << "angle " << (angle * (180 / CV_PI));
        cv::Mat rotation = cv::Mat(2, 2, CV_32F);
        rotation.at<float>(0, 0) = std::cos(angle);
        rotation.at<float>(0, 1) = -std::sin(angle);
        rotation.at<float>(1, 0) = std::sin(angle);
        rotation.at<float>(1, 1) = std::cos(angle);
        cv::Mat point1 = cv::Mat(2, 1, CV_32F);
        point1.at<float>(0, 0) = linePoints[0].x;
        point1.at<float>(1, 0) = linePoints[0].y;
        cv::Mat point2 = cv::Mat(2, 1, CV_32F);
        point2.at<float>(0, 0) = linePoints[1].x;
        point2.at<float>(1, 0) = linePoints[1].y;
        cv::Mat point1Res = rotation * point1;
        cv::Mat point2Res = rotation * point2;
        if (point1Res.at<float>(1, 0) < point2Res.at<float>(1, 0)) {
            cv::Point temp = linePoints[0];
            linePoints[0] = linePoints[1];
            linePoints[1] = temp;
        }
    }

    // Paint largest lines
    for (int i = 0; i < 2; ++i)
    {
        cv::line(dst, cv::Point(lines[largestLines[i]][0], lines[largestLines[i]][1]), cv::Point(lines[largestLines[i]][2], lines[largestLines[i]][3]), Scalar(0, 0, 255), 1);
        cv::circle(dst, linePoints[i], 5, colors[i], 2);
    }

    return true;

}

bool PlaneDetector::findPlanesBinaryDistinctColors(cv::Mat &dst)
{
    // First, color binary image
    cv::Mat HSVImage;
    std::vector<cv::Mat> HSVChannels;
    TrackingCommon::frame.convertTo(HSVImage, CV_BGR2HSV);
    cv::split(HSVImage, HSVChannels);
    // Line will be fully saturated
    cv::Mat binaryImage = HSVChannels[2];

    cv::threshold(binaryImage, binaryImage, TrackingCommon::colorHSVThreshold, 250, cv::THRESH_BINARY);

    // Apply canny edge detector
    //cv::Canny(binaryImage, binaryImage, 30, 100);
    //cv::imshow("CannyE", binaryImage);
    //return false;

    //cv::Mat kernel = cv::Mat(3, 3, CV_32F, cv::Scalar(1));
    //cv::erode(binaryImage, binaryImage, kernel);
    cv::Scalar colors[4];
    colors[0] = cv::Scalar(0, 0, 255);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(255, 0, 0);
    colors[3] = cv::Scalar(128, 128, 0);

    //////////////////////////////////////////////
    // Find contours
    //////////////////////////////////////////////

    std::vector<std::vector<cv::Point> > contours, contoursFiltered;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    //////////////////////////////////////////////
    // Reduce small areas
    //////////////////////////////////////////////

    std::vector<double> contourAreas;
    cv::Mat currentMaskOfImage = Mat::zeros(binaryImage.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        //double perimeter = cv::arcLength(contours[i], true);

        if (area > TrackingCommon::areaThreshold && area < TrackingCommon::areaMaxThreshold)
        {
            currentMaskOfImage.setTo(0);
            cv::drawContours(currentMaskOfImage, contours, i, cv::Scalar(255), -1);

            cv::Scalar mean = cv::mean(TrackingCommon::frame, currentMaskOfImage);
            double colorMean = (mean[0] + mean[1] + mean[2]) / 3.0;
            if (colorMean < TrackingCommon::colorBrightnessThreshold)
            {
                contoursFiltered.push_back(contours[i]);
            }
        }
    }

    if (contoursFiltered.size() != 2) {
        qDebug() << contoursFiltered.size() << " areas were found. Discarding.";
        for (int i = 0; i < contoursFiltered.size(); ++i)
        {
            cv::drawContours(dst, contoursFiltered, i, cv::Scalar(0, 255), -1);
        }
        return false;
    }

    cv::Mat maskOfImage[2];
    for (int i = 0; i < 2; ++i)
    {
        maskOfImage[i] = Mat::zeros(binaryImage.size(), CV_8UC1);
        cv::drawContours(maskOfImage[i], contoursFiltered, i, cv::Scalar(255), -1);
    }
    //cv::imshow("Contour0", maskOfImage[0]);
    //cv::imshow("Contour1", maskOfImage[1]);

    //////////////////////////////////////////////
    // Filter areas by color
    //////////////////////////////////////////////

    cv::Scalar averageColors[2];
    for (int i = 0; i < 2; ++i)
    {
        averageColors[i] = cv::mean(TrackingCommon::frame, maskOfImage[i]);
    }
    // Take the one with the most red, as the right one
    int rightBandIndex = 1;
    if (averageColors[1][1] > averageColors[0][1])
    {
        rightBandIndex = 0;
    }
    //cv::drawContours(dst, contoursFiltered, rightBandIndex, cv::Scalar(255), -1);

    //////////////////////////////////////////////
    // find the rectangles found
    //////////////////////////////////////////////

    cv::RotatedRect bands[2];
    std::vector<cv::Vec4i> lines;
    for (int i = 0; i < 2; ++i)
    {
        bands[i] = cv::minAreaRect(contoursFiltered[i]);
        //cv::ellipse(dst, bands[i], cv::Scalar(255, 0, 0), 2);
        cv::Point2f vertices[4];

        bands[i].points(vertices);
        cv::Vec4i line;
        if (bands[i].size.width > bands[i].size.height)
        {
            line[0] = (vertices[0].x + vertices[1].x) / 2.0;
            line[1] = (vertices[0].y + vertices[1].y) / 2.0;
            line[2] = (vertices[2].x + vertices[3].x) / 2.0;
            line[3] = (vertices[2].y + vertices[3].y) / 2.0;
        }
        else
        {
            line[0] = (vertices[1].x + vertices[2].x) / 2.0;
            line[1] = (vertices[1].y + vertices[2].y) / 2.0;
            line[2] = (vertices[3].x + vertices[0].x) / 2.0;
            line[3] = (vertices[3].y + vertices[0].y) / 2.0;
        }
        lines.push_back(line);
    }

    //////////////////////////////////////////////
    // Find vanishing point
    //////////////////////////////////////////////

    double xVanishing, yVanishing;
    int largestLines[2]; largestLines[0] = 0; largestLines[1] = 1;
    cv::Point2f vanishingPoint = calculateVanishingPoint(lines, largestLines);
    if (vanishingPoint.x == 0.0 && vanishingPoint.y == 0.0)
        return false;
    xVanishing = vanishingPoint.x;
    yVanishing = vanishingPoint.y;

    //////////////////////////////////////////////
    // Get solution pairs by rearraging lines, so that x1, y1 and both lines be closer to vanishing point
    //////////////////////////////////////////////

    for (int i = 0; i < lines.size(); ++i)
    {
        double vanishingDistance1 = std::sqrt(std::pow(lines[i][0] - xVanishing, 2) + std::pow(lines[i][1] - yVanishing, 2));
        double vanishingDistance2 = std::sqrt(std::pow(lines[i][2] - xVanishing, 2) + std::pow(lines[i][3] - yVanishing, 2));
        if (vanishingDistance1 < vanishingDistance2)
        {
            // swap points
            cv::Vec4i temp = lines[i];
            lines[i][0] = temp[2];
            lines[i][1] = temp[3];
            lines[i][2] = temp[0];
            lines[i][3] = temp[1];
        }
    }

    //////////////////////////////////////////////
    // Calculate homography
    //////////////////////////////////////////////

    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;
    srcPoints.push_back(cv::Point2f(0, 0));
    srcPoints.push_back(cv::Point2f(TrackingCommon::linesHeight, 0));
    srcPoints.push_back(cv::Point2f(TrackingCommon::linesHeight, TrackingCommon::linesWidth - TrackingCommon::lineWidth));
    srcPoints.push_back(cv::Point2f(0, TrackingCommon::linesWidth - TrackingCommon::lineWidth));
    if (rightBandIndex == 0)
    {
        dstPoints.push_back(cv::Point2f(lines[0][2], lines[0][3]));
        dstPoints.push_back(cv::Point2f(lines[0][0], lines[0][1]));
        dstPoints.push_back(cv::Point2f(lines[1][0], lines[1][1]));
        dstPoints.push_back(cv::Point2f(lines[1][2], lines[1][3]));
    }
    else
    {
        dstPoints.push_back(cv::Point2f(lines[1][2], lines[1][3]));
        dstPoints.push_back(cv::Point2f(lines[1][0], lines[1][1]));
        dstPoints.push_back(cv::Point2f(lines[0][0], lines[0][1]));
        dstPoints.push_back(cv::Point2f(lines[0][2], lines[0][3]));
    }
    foundHomography = cv::findHomography(srcPoints, dstPoints);

    HomographyDecomposer decomposer;
    decomposer.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);
    decomposer.setHomography(foundHomography);
    decomposer.setMethod(HomographyDecomposer::ROTATIONAL);
    decomposer.decompose();

    // If reconstruction generated a negatize Z axis, then we should use the other solution
    if (decomposer.transform.at<double>(2, 2) > 0)
    {
        dstPoints.clear();
        if (rightBandIndex == 0)
        {
            dstPoints.push_back(cv::Point2f(lines[0][0], lines[0][1]));
            dstPoints.push_back(cv::Point2f(lines[0][2], lines[0][3]));
            dstPoints.push_back(cv::Point2f(lines[1][2], lines[1][3]));
            dstPoints.push_back(cv::Point2f(lines[1][0], lines[1][1]));
        }
        else
        {
            dstPoints.push_back(cv::Point2f(lines[1][0], lines[1][1]));
            dstPoints.push_back(cv::Point2f(lines[1][2], lines[1][3]));
            dstPoints.push_back(cv::Point2f(lines[0][2], lines[0][3]));
            dstPoints.push_back(cv::Point2f(lines[0][0], lines[0][1]));
        }
        foundHomography = cv::findHomography(srcPoints, dstPoints);
    }

    //////////////////////////////////////////////
    // 3D Reconstruction
    //////////////////////////////////////////////

    cv::Mat foundHomography32F = cv::Mat(3, 3, CV_32F);
    for (int i = 0; i < 9; ++i)
        foundHomography32F.at<float>(i / 3, i % 3) = foundHomography.at<double>(i / 3, i % 3);

    if (!filter.isInited())
    {
        filter.initialize(3, 10);
        filter.setState(foundHomography32F);
    }
    filter.filter(foundHomography32F);

    for (int i = 0; i < 9; ++i)
        foundHomography.at<double>(i / 3, i % 3) = foundHomography32F.at<float>(i / 3, i % 3);

    decomposer.setHomography(foundHomography);
    decomposer.decompose();

    // Get the translation for the center of the template
    cv::Mat templateCenter = cv::Mat(4, 4, CV_64F);
    cv::setIdentity(templateCenter);
    templateCenter.at<double>(0, 3) = ((cv::Point2f)srcPoints[2]).x / 2;
    templateCenter.at<double>(1, 3) = ((cv::Point2f)srcPoints[2]).y / 2;
    //cv::gemm(decomposer.transform, templateCenter, 1.0, cv::Mat(), 0, decomposer.transform);

    //////////////////////////////////////////////
    // Define a custom "ZERO"
    //////////////////////////////////////////////
    double xAngle;
    double yAngle;
    cv::Mat Rx;
    cv::Mat Ry;
    cv::Mat R_inv;

    if (TrackingCommon::markObjectZero || TrackingCommon::markObjectRotZero)
    {
        cv::Mat currentTrans;
        if (TrackingCommon::markObjectRotZero)
        {
            currentTrans = ((TrackingCommon::objectZero)(cv::Range(0, 3), cv::Range(3, 3))).clone();
        }
        decomposer.transform.copyTo(TrackingCommon::objectZero);

        // Camera pan tilt
        xAngle = ( 3.1416 / 12 ) * TrackingCommon::pTZCameraController.tilt / (7789);
        yAngle = ( 3.1416 / 6 ) * TrackingCommon::pTZCameraController.pan / (15578);
        Rx = TrackingCommon::buildRotationMatrix(0, -xAngle);
        Ry = TrackingCommon::buildRotationMatrix(1, -yAngle);

        cv::invert(TrackingCommon::objectZero, TrackingCommon::objectZero);
        cv::invert(Rx * Ry, R_inv);
        cv::gemm(TrackingCommon::objectZero, R_inv, 1.0, cv::Mat(), 0, TrackingCommon::objectZero);

        if (TrackingCommon::markObjectRotZero)
        {
            cv::Mat newTrans = (TrackingCommon::objectZero)(cv::Range(0, 3), cv::Range(3, 3));
            currentTrans.copyTo(newTrans);
        }
        TrackingCommon::markObjectZero = false;
        TrackingCommon::markObjectRotZero = false;
    }

    xAngle = ( 3.1416 / 12 ) * TrackingCommon::pTZCameraController.tilt / (7789);
    yAngle = ( 3.1416 / 6 ) * TrackingCommon::pTZCameraController.pan / (15578);
    Rx = TrackingCommon::buildRotationMatrix(0, -xAngle);
    Ry = TrackingCommon::buildRotationMatrix(1, -yAngle);

    cv::gemm(TrackingCommon::objectZero * Rx * Ry, decomposer.transform, 1.0, cv::Mat(), 0, decomposer.transform);

    decomposer.transform.copyTo(foundTransform);

    cv::Mat rotation = (foundTransform)(Range(0, 3), Range(0, 3));
    cv::Mat angles = cv::Mat(3, 1, CV_64F);
    cv::Rodrigues(rotation, angles);
    cv::Mat debugData = cv::Mat(6, 1, CV_64F);
    double toDeg = 180.0 / CV_PI;
    debugData.at<double>(0, 0) = angles.at<double>(0, 0) * toDeg;
    debugData.at<double>(1, 0) = angles.at<double>(1, 0) * toDeg;
    debugData.at<double>(2, 0) = angles.at<double>(2, 0) * toDeg;

    if (TrackingCommon::totalWindows > 0 && TrackingCommon::positionEstimatorController.isReady())
    {
        MeanShiftTracker* tracker = (MeanShiftTracker*)TrackingCommon::trackingWindows[TrackingCommon::totalWindows - 1];
        std::vector<cv::Point2f> srcCorners(1);
        std::vector<cv::Point2f> dstCorners(1);
        srcCorners[0] = cv::Point2f(TrackingCommon::linesHeight / 2.0, (TrackingCommon::linesWidth - TrackingCommon::lineWidth) / 2.0);
        cv::perspectiveTransform(srcCorners, dstCorners, foundHomography);
        tracker->updateTrackingWindowFromCenter((cv::Point2f) dstCorners[0]);

        cv::Mat displayData = TrackingCommon::calculate2DPosition((cv::Point2f) dstCorners[0]);
        cv::Point3f pointReal;
        pointReal.x = displayData.at<float>(0, 11);
        pointReal.y = displayData.at<float>(0, 12);
        pointReal.z = displayData.at<float>(0, 13);

        debugData.at<double>(3, 0) = pointReal.x;
        debugData.at<double>(4, 0) = pointReal.y;
        debugData.at<double>(5, 0) = 0.0;
    }
    else
    {
        debugData.at<double>(3, 0) = foundTransform.at<double>(0, 3);
        debugData.at<double>(4, 0) = foundTransform.at<double>(1, 3);
        debugData.at<double>(5, 0) = foundTransform.at<double>(2, 3);
    }

    if (TrackingCommon::searchPlaneLinesRecord)
    {
        TrackingCommon::mainController->displayData(std::string("3DPose"), debugData, DataDisplayer::APPEND);
    }

    if (TrackingCommon::gpSerialPort->isOpen() && frameSkip >= 0)
    {
        std::stringstream commandBuilder;
        commandBuilder << "!98,";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << angles.at<double>(0, 0) * toDeg << ",";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << angles.at<double>(1, 0) * toDeg << ",";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << angles.at<double>(2, 0) * toDeg << ",";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << debugData.at<double>(3, 0) << ",";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << debugData.at<double>(4, 0) << ",";
        commandBuilder << std::setw(11) << std::setprecision(6) << std::fixed << debugData.at<double>(5, 0) << "+";
        qDebug() << "Char: " << commandBuilder.str().c_str();
        QByteArray text = QByteArray(commandBuilder.str().c_str());
        TrackingCommon::gpSerialPort->write(text);
        frameSkip = 0;
    }
    frameSkip++;

    // Paint largest lines
    for (int i = 0; i < 2; ++i)
    {
        cv::line(dst, cv::Point(lines[largestLines[i]][0], lines[largestLines[i]][1]), cv::Point(lines[largestLines[i]][2], lines[largestLines[i]][3]), Scalar(0, 0, 255), 2);
    }

    return true;
}

cv::Point2f PlaneDetector::calculateVanishingPoint(std::vector<cv::Vec4i> lines, int largestLines[2])
{
    double m[2], b[2];
    for (int i = 0; i < 2; ++i)
    {
        double dx = (lines[largestLines[i]][2] - lines[largestLines[i]][0]);
        if (dx == 0.0)
        {
            dx = 0.00000001;
        }
        m[i] = ((double)(lines[largestLines[i]][3] - lines[largestLines[i]][1])) / dx;
        b[i] = lines[largestLines[i]][1] - m[i] * lines[largestLines[i]][0];
    }
    if (m[0] == 0 && m[1] == 0.0)
    {
        qDebug() << "Both slopes are 0 on vanishing line";
        return cv::Point2f(0, 0);
    }
    double xVanishing = (b[1] - b[0]) / (m[0] - m[1]);
    double yVanishing = m[0] * xVanishing + b[0];

    // For each segment, find which is closest to vanishing point
    for (int i = 0; i < 2; ++i)
    {
        double vanishingDistance1 = std::sqrt(std::pow(lines[largestLines[i]][0] - xVanishing, 2) + std::pow(lines[largestLines[i]][1] - yVanishing, 2));
        double vanishingDistance2 = std::sqrt(std::pow(lines[largestLines[i]][2] - xVanishing, 2) + std::pow(lines[largestLines[i]][3] - yVanishing, 2));
        if (vanishingDistance1 < vanishingDistance2)
            linePoints[i] = cv::Point(lines[largestLines[i]][0], lines[largestLines[i]][1]);
        else
            linePoints[i] = cv::Point(lines[largestLines[i]][2], lines[largestLines[i]][3]);
    }

    return cv::Point2f(xVanishing, yVanishing);
}

bool PlaneDetector::estimateDenseFlowGPU(cv::Mat &flwX, cv::Mat &flwY)
{
    if (images.size() < 2) {
        qDebug() << "FlowHomography - Not enough images for flow detection";
        return false;
    }
    cv::Mat srcNxt = images[images.size() - 1];
    cv::Mat srcPrv = images[0];

    cv::Mat prv = cv::Mat(srcPrv.size(), CV_8UC1);
    cv::Mat nxt = cv::Mat(srcNxt.size(), CV_8UC1);

    cv::cvtColor(srcPrv, prv, CV_BGR2GRAY);
    cv::cvtColor(srcNxt, nxt, CV_BGR2GRAY);

    cv::Mat prvS = cv::Mat(srcPrv.size(), CV_32FC1);
    cv::Mat nxtS = cv::Mat(srcNxt.size(), CV_32FC1);

    prv.convertTo(prvS, prvS.type());
    nxt.convertTo(nxtS, nxtS.type());

    flwX = cv::Mat(srcPrv.size(), CV_32FC1);
    flwY = cv::Mat(srcPrv.size(), CV_32FC1);
    cv::gpu::GpuMat gPrv(prvS);
    cv::gpu::GpuMat gNxt(nxtS);
    cv::gpu::GpuMat gFlwX;
    cv::gpu::GpuMat gFlwY;

    //cv::gpu::BroxOpticalFlow broxOpticalFlow = cv::gpu::BroxOpticalFlow(0.048, 0.95, 1.0, 1, 2, 5);
    //cv::gpu::BroxOpticalFlow broxOpticalFlow = cv::gpu::BroxOpticalFlow(0.048, 0.95, 1.0, 2, 6, 6);
    cv::gpu::BroxOpticalFlow broxOpticalFlow = cv::gpu::BroxOpticalFlow(0.024, 0.95, 1.0, 2, 10, 10);
    broxOpticalFlow(gPrv, gNxt, gFlwX, gFlwY);

    gFlwX.download(flwX);
    gFlwY.download(flwY);

    return true;
}

bool PlaneDetector::estimateDenseFlowCPU(cv::Mat &flwX, cv::Mat &flwY)
{
    if (images.size() < 2) {
        qDebug() << "FlowHomography - Not enough images for flow detection";
        return false;
    }
    cv::Mat srcNxt = images[images.size() - 1];
    cv::Mat srcPrv = images[0];

    cv::Mat prv = cv::Mat(srcPrv.size(), CV_8UC1);
    cv::Mat nxt = cv::Mat(srcNxt.size(), CV_8UC1);

    cv::cvtColor(srcPrv, prv, CV_BGR2GRAY);
    cv::cvtColor(srcNxt, nxt, CV_BGR2GRAY);

    cv::Mat prvS = cv::Mat(srcPrv.size(), CV_32FC1);
    cv::Mat nxtS = cv::Mat(srcNxt.size(), CV_32FC1);

    prv.convertTo(prvS, prvS.type());
    nxt.convertTo(nxtS, nxtS.type());

    flwX = cv::Mat(srcPrv.size(), CV_32FC1);
    flwY = cv::Mat(srcPrv.size(), CV_32FC1);

    cv::Mat flw = cv::Mat(srcPrv.size(), CV_32FC2);

    if (true)
    {
        //calcOpticalFlowFarneback(prv, nxt, flw, 0.5, 3, 15, 3, 5, 1.2, 0);
        calcOpticalFlowFarneback(prv, nxt, flw, 0.5, 3, 7, 15, 5, 1.2, 0);
        std::vector<cv::Mat> flwDims;
        flwDims.push_back(flwX);
        flwDims.push_back(flwY);
        cv::split(flw, flwDims);
    }
    else
    {
        Mat features;
        goodFeaturesToTrack(prv, features, 3000, 0.01, 4);
        Mat status;
        Mat error;
        calcOpticalFlowPyrLK(prv, nxt, features, flw, status, error);
    }

//    cv::Mat image = TrackingCommon::testsController.tests[TrackingCommon::testsController.currentTestIndex].images[1].clone();
//    drawOptFlowMap(flw, image, 16, 1.5, cv::Scalar(0, 255, 0));
//    cv::imshow("dst", image);

   return true;
}

void PlaneDetector::drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double scale, const Scalar& color)
{
    for (int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

bool PlaneDetector::filterKeypointsWithDenseFlow(std::vector<cv::KeyPoint> &keypoints, std::vector<cv::KeyPoint> &keypointsFiltered, cv::Mat &flwX, cv::Mat &flwY, cv::Mat &dst, float motionThreshold)
{
    int rows = dst.rows;
    int cols = dst.cols;

    // Let's filter out features that are not close to a moving region
    uchar* referencePointerX;
    uchar* referencePointerY;
    uchar* targetPointer;

    // 1, create a image of the motion
    cv::Mat motionImage = cv::Mat(dst.size(), CV_8UC1);
    motionImage.setTo(0);
    for (int y = 0; y < rows; ++y)
    {
        referencePointerX = flwX.ptr(y);
        referencePointerY = flwY.ptr(y);
        targetPointer = motionImage.ptr(y);
        for (int x = 0; x < cols; ++x)
        {
            float dx = *((float*) (referencePointerX + 4 * x));
            float dy = *((float*) (referencePointerY + 4 * x));
            //*(targetPointer + x) = cv::saturate_cast<uchar>(dx * dx + dy * dy);
            if (dx * dx + dy * dy > 20) {
                //qDebug() << "Motion: (" << x << "," << y << ") + (" << dx << "," << dy << ")";
                //motionImage.at<uchar>(x + std::floor(dx), y + std::floor(dy)) = 1;
                int targetX = x + std::floor(dx);
                if (targetX < 0)
                    targetX = 0;
                else if (targetX >= cols)
                    targetX >= cols - 1;

                int targetY = y + std::floor(dy);
                if (targetY < 0)
                    targetY = 0;
                else if (targetY >= rows)
                    targetY = rows- 1;

                motionImage.at<uchar>(targetY, targetX) = 1;
            }
        }
    }

    // 2, expand and shrink regions
    cv::Mat motionImageDilated;
    cv::Mat motionImageEroded;
    cv::Mat expandKernel = cv::Mat(7, 7, CV_32F, cv::Scalar(1));
    cv::dilate(motionImage, motionImageDilated, expandKernel, cv::Point(-1, -1), 3);
    cv::erode(motionImageDilated, motionImageEroded, expandKernel, cv::Point(-1, -1), 5);

    for (int y = 0; y < rows; ++y)
    {
        targetPointer = motionImageEroded.ptr(y);
        for (int x = 0; x < cols; ++x)
        {
            if (*((uchar*) (targetPointer + x)) > 0)
                cv::circle(dst, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);
        }
    }

    // 4, filter descriptors
    for (int i = 0; i < keypoints.size(); ++i)
    {
        cv::KeyPoint keypoint = keypoints[i];
        if (motionImage.at<unsigned char>(keypoint.pt) > 0 || true) {
            //qDebug() << "Found keypoint";
            keypointsFiltered.push_back(keypoint);
        }
    }

    // 4-1, return if not enough points
    if (keypointsFiltered.size() < 20) {
        qDebug() << "Not enough keypoints";
        return false;
    }

    return true;
}
