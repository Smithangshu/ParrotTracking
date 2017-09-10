#include "CuboidTrackerMultipleFace.h"
#include "common/TrackingCommon.h"
#include <QDebug>

CuboidTrackerMultipleFace::CuboidTrackerMultipleFace()
{
    type = TYPE_CUBOID_SINGLE_FACE;
    qDebug() << "Tracking Multiple Faces";
    iterationsAfterFaceChange = 10000000;

    for (int i = 0; i < 6; ++i) {
        trackFace[i] = false;
        trackedFaceConverged[i] = true;
    }

    disableFaceChangeLock = false;
    reconstructionAverage = cv::Mat::zeros(4, 4, CV_64F);
    firstIteration = true;
    forceTrackIndex = -1;
}

int CuboidTrackerMultipleFace::calculateFaceWithLeastError()
{
    int bestIndex = -1;
    double currentError = DBL_MAX;
    for (int i = 0; i < 6; ++i)
    {
        if (faceIsVisible[i])
        {
            cv::warpPerspective(trackers[i].I, trackers[i].I_w, trackers[i].W_3x3, trackers[i].I_w.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
            double newError = trackers[i].computeErrorImage(trackers[i].T, trackers[i].I_w, trackers[i].error_image);
            if (newError < currentError)
            {
                currentError = newError;
                bestIndex = i;
            }
        }
    }
    //std::cout << "\n";
    return face;
    return bestIndex;
}

int CuboidTrackerMultipleFace::calculateFaceWithNormalMostAligned()
{
    cv::Mat centerTranslation = centerTransform.col(3);
    int bestIndex = -1;
    double currentValue = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        if (faceIsVisible[i])
        {
            //qDebug() << "Visible face: " << i;
            double newValue = centerTranslation.dot(normals[i]);
            if (currentValue < newValue)
            {
                currentValue = newValue;
                bestIndex = i;
            }

            // Added to select more than one face as visible
            // TODO: May be this should be in a different method
            float angle =  std::acos(newValue / (TrackingCommon::lengthOfVector(centerTranslation) * TrackingCommon::lengthOfVector(normals[i])));
            //if (angle < CV_PI / 2.7)
            if (std::abs(angle) < CV_PI / 2.7)
            {
                trackFace[i] = true;
            }
            else
            {
                trackFace[i] = false;
            }
        }
        else
        {
            // Make sure no invisible faces remain visible
            trackFace[i] = false;
        }
    }

    if (lockFace)
        return face;

    return bestIndex;
}

void CuboidTrackerMultipleFace::trackObject(cv::Mat &newImage)
{
    maximumTrackedFaces = TrackingCommon::maxTrackedFaces;

    // Sync camera calibration
    homographyDecomposer.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);

    if (firstIteration)
    {
        // Single object tracking and reconstruction
        trackers[face].trackObject(newImage);
        calculateTransformationFromHomography();
        calculateAllTransformationsFromSingleTransformation();
        calculateHomographiesFromTransformation();
        updateNormals();
        calculateVisibleFaces();
        calculateFaceWithNormalMostAligned();

        // Get an initial guess for homographies
        for (int i = 0; i < 6; ++i)
            currentHomographies[i] = trackers[i].W_3x3.clone();

        firstIteration = false;
    }

    // Store the current homography
    for (int i = 0; i < 6; ++i)
    {
        if (trackFace[i] == false)
            currentHomographies[i] = trackers[i].W_3x3.clone();
    }

    if (this->cuboidTrackingMode == CuboidTracker::MULTI_VIEW)
    {
        // For locked face, store the current face
        int currentFace = face;

        // Two or more faces are going to be used
        //   Generate a composed 3D by averaging poses. Averaging factor is proportional to feasibility of 3D reconstruction
        // For each Face
        // Generate alternate reconstructions
        for (int i = 0; i < 6; ++i)
        {
            if (trackFace[i])
            {
                //qDebug() << "Face " << i << " will be tracked";
                setBaseTracker(i);

                // Generate a image for storing image
                reconstructionImages[i] = TrackingCommon::frame.clone();

                // Restore the homography for tracking
                trackers[i].setHomography(currentHomographies[i], false);
                // Track each face in 2D
                trackers[i].trackObject(newImage);

                // Calculate the 3D reconstruction
                calculateTransformationFromHomography();
                // We define the decomposition error as the absolute value of the decomposition error
                // Since it's the dot product of two unitary vectors, the abs value goes from 0 to 1
                // We take the inverse since the least error is when they're orthogonal
                faceReconstructionError[i] = std::abs(homographyDecomposer.decompositionError);
                calculateAllTransformationsFromSingleTransformation();
                // Determine how well 3D reconstruction is
                calculateHomographiesFromTransformation();
                // Record the current reconstruction for later merging
                cuboidReconstructions[i] = centerTransform.clone();

                // Save permanently the last good tracking
                currentHomographies[i] = trackers[i].W_3x3.clone();
                // Indicate that tracked face converged  (so do not take into
                // account that when merging
                trackedFaceConverged[i] = trackers[i].converged;

                // Draw the reconstruction
                highlightObject(reconstructionImages[i]);

                //std::string winNameT = "T "; winNameT += '0' + i; TrackingCommon::mainController->displayImageWindowCV(winNameT, trackers[i].T);
                //std::string winNameW = "W "; winNameW += '0' + i; TrackingCommon::mainController->displayImageWindowCV(winNameW, trackers[i].warpedImages[0]);
                //std::string tabName = "Decomp "; tabName += '0' + i; TrackingCommon::mainController->displayData(tabName, homographyDecomposer.transform, DataDisplayer::FULL);
                //std::string winName = "Reconstruction "; winName += '0' + i; TrackingCommon::mainController->displayImageWindowCV(winName, reconstructionImages[i]);
            }
        }

        cv::Mat debugFaceMerge = cv::Mat(14, 7, CV_32F);
        debugFaceMerge.setTo(0);

        // Blend the reconstructions
        int totalActive = 0;
        float totalReconstructionFactor = 0;
        float angleXSum = 0;
        float angleYSum = 0;
        float angleZSum = 0;
        float txSum = 0;
        float tySum = 0;
        float tzSum = 0;

        for (int i = 0; i < 6; ++i)
            if (trackFace[i] && trackedFaceConverged[i])
                ++totalActive;

        if (totalActive < 1)
        {
            if (forceTrackIndex == -1)
            {
                // If there are no active face, take the one with the least error
                double maxError = 1000000;
                for (int i = 0; i < 6; ++i)
                {
                    if (trackFace[i])
                    {
                        if (trackers[i].lastAverageError < maxError)
                        {
                            maxError = trackers[i].lastAverageError;
                            forceTrackIndex = i;
                        }
                    }
                    // This will disable tracking on all faces
                    trackFace[i] = false;
                }
                iterationsAfterFaceChange = 1;
                qDebug() << "Entering recovering mode with face " << forceTrackIndex;
            }
            // This indicates we are in recovery mode
            if (iterationsAfterFaceChange > 0)
            {
                for (int i = 0; i < 6; ++i)
                {
                    trackFace[i] = false;
                    trackedFaceConverged[i] = false;
                }
                trackFace[forceTrackIndex] = true;
                trackedFaceConverged[forceTrackIndex] = true;
            }
            qDebug() << "iterationsAfterFaceChange:  " << iterationsAfterFaceChange;
        }

        totalActive = 0;
        std::vector<cv::Mat> quaternions;
        for (int i = 0; i < 6; ++i)
        {
            debugFaceMerge.at<float>(0, i) = trackFace[i] ? 1 : 0;
            debugFaceMerge.at<float>(1, i) = trackedFaceConverged[i] ? 1 : 0;
            if (trackFace[i] && trackedFaceConverged[i])
            {
                if (forceTrackIndex >= 0)
                    ++iterationsAfterFaceChange;

                if (iterationsAfterFaceChange > 20)
                {
                    forceTrackIndex = -1;
                    iterationsAfterFaceChange = 0;
                    qDebug() << "Leaving recovering mode";
                }
                // Averaging factor
                float factor = 1 - faceReconstructionError[i];
                totalReconstructionFactor += factor;
                ++totalActive;

                // Build an estimate
                //std::string tabName = "Rcons "; tabName += '0' + i; TrackingCommon::mainController->displayData(tabName, cuboidReconstructions[i], DataDisplayer::FULL);
                float angleX, angleY, angleZ, tx, ty, tz;
                tx = cuboidReconstructions[i].at<double>(0, 3);
                ty = cuboidReconstructions[i].at<double>(1, 3);
                tz = cuboidReconstructions[i].at<double>(2, 3);
                TrackingCommon::getAnglesFromTransform(cuboidReconstructions[i], angleX, angleY, angleZ);

                // Increment with factor
                angleXSum += factor * angleX;
                angleYSum += factor * angleY;
                angleZSum += factor * angleZ;
                txSum += factor * tx;
                tySum += factor * ty;
                tzSum += factor * tz;

                // Print: Rotations
                debugFaceMerge.at<float>(2, i) = angleX;
                debugFaceMerge.at<float>(3, i) = angleY;
                debugFaceMerge.at<float>(4, i) = angleZ;

                // Traslations
                debugFaceMerge.at<float>(5, i) = tx;
                debugFaceMerge.at<float>(6, i) = ty;
                debugFaceMerge.at<float>(7, i) = tz;

                // Decomposition Error
                debugFaceMerge.at<float>(8, i) = faceReconstructionError[i];

                //Quaternion Tests
                cv::Mat quaternion = TrackingCommon::getQuaternionFromTransform(cuboidReconstructions[i]);
                quaternions.push_back(quaternion);

                TrackingCommon::getEulerAnglesFromTransform(cuboidReconstructions[i], angleX, angleY, angleZ);
                debugFaceMerge.at<float>(10, i) = quaternion.at<double>(0, 0);
                debugFaceMerge.at<float>(11, i) = quaternion.at<double>(1, 0);
                debugFaceMerge.at<float>(12, i) = quaternion.at<double>(2, 0);
                debugFaceMerge.at<float>(13, i) = quaternion.at<double>(3, 0);
            }
        }

        angleXSum /= totalReconstructionFactor;
        angleYSum /= totalReconstructionFactor;
        angleZSum /= totalReconstructionFactor;
        txSum /= totalReconstructionFactor;
        tySum /= totalReconstructionFactor;
        tzSum /= totalReconstructionFactor;

        // Build the new transformation matrix
        cv::Mat averageQuaternion = TrackingCommon::averageQuaternions(quaternions);
        cv::Mat composedTransform = TrackingCommon::getTransformFromQuaternion(averageQuaternion);

        //cv::Mat composedTransform = TrackingCommon::getTransformFromAngles(angleXSum, angleYSum, angleZSum);
        composedTransform.at<double>(0, 3) = txSum;
        composedTransform.at<double>(1, 3) = tySum;
        composedTransform.at<double>(2, 3) = tzSum;
        composedTransform.copyTo(centerTransform);

        //TrackingCommon::mainController->displayData(std::string("composedTransform"), composedTransform, DataDisplayer::APPEND);

        // Draw composed cuboid

        // Update 3D Tracker
        calculateAllTransformationsFromSingleTransformation(true);
        calculateHomographiesFromTransformation(false);
        updateNormals();
        if (forceTrackIndex == -1)
        {
            calculateVisibleFaces();
            calculateFaceWithNormalMostAligned();
        }

        cv::Mat mergedImage = TrackingCommon::frame.clone();
        highlightObject(mergedImage);
        TrackingCommon::mainController->displayImageWindowCV(std::string("Merged"), mergedImage);

        if (lockFace)
        {
            face = currentFace;
            trackers[face].setHomography(currentHomographies[face], false);
            calculateTransformationFromHomography();
            calculateAllTransformationsFromSingleTransformation();
            calculateHomographiesFromTransformation();
            updateNormals();
            calculateVisibleFaces();
            calculateFaceWithNormalMostAligned();
        }

        centerTransformAlternate = cuboidReconstructions[face].clone();
        centerTransformFixed = centerTransform.clone();

        // Let's check faces that diverged, to we reset tracking for them
        for (int i = 0; i < 6; ++i)
        {
            if (trackFace[i] && !trackedFaceConverged[i])
            {
                homographies[i].convertTo(currentHomographies[i], CV_32F);
                trackers[i].setHomography(homographies[i]);
            }
        }

        debugFaceMerge.at<float>(2, 6) = angleXSum;
        debugFaceMerge.at<float>(3, 6) = angleYSum;
        debugFaceMerge.at<float>(4, 6) = angleZSum;
        debugFaceMerge.at<float>(5, 6) = txSum;
        debugFaceMerge.at<float>(6, 6) = tySum;
        debugFaceMerge.at<float>(7, 6) = tzSum;
        //TrackingCommon::mainController->displayData(std::string("FaceMerge"), debugFaceMerge, DataDisplayer::FULL);
    }
    else
    {
        ESMTracker* tracker = getTracker(0);
        tracker->trackObject(newImage);
        calculateTransformationFromHomography();
        calculateAllTransformationsFromSingleTransformation();
    }
}

void CuboidTrackerMultipleFace::highlightObject(cv::Mat &targetImage)
{
    for (int i = 0; i < 6; ++i)
    {
        if (i != face && !trackFace[i] || TrackingCommon::maxTrackedFaces == 1)
            trackers[i].highlightColor = cv::Scalar(255, 0, 0);

        trackers[i].highlightObject(targetImage);
    }
    if (TrackingCommon::maxTrackedFaces > 1)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (i != face && trackFace[i])
                trackers[i].highlightColor = cv::Scalar(48, 128, 255);

            trackers[i].highlightObject(targetImage);
        }
    }

    trackers[face].highlightColor = cv::Scalar(0, 0, 255);
    trackers[face].highlightObject(targetImage);
}

void CuboidTrackerMultipleFace::setMaximumTrackedFaces(int maximumTrackedFaces)
{
    TrackingCommon::maxTrackedFaces = maximumTrackedFaces;
}

void CuboidTrackerMultipleFace::setFaceChangeLock(bool disable)
{
    this->disableFaceChangeLock = disable;
}
