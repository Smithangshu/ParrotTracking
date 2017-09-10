#ifndef CUBOIDTRACKERMULTIPLEFACE_H
#define CUBOIDTRACKERMULTIPLEFACE_H

#include "CuboidTracker.h"

class CuboidTrackerMultipleFace : public CuboidTracker {
private:

    // Prevent race between faces
    int iterationsAfterFaceChange;

    // For tracking two or more faces at a time
    int trackFace[6];
    int trackedFaceConverged[6];
    double faceReconstructionError[6];
    int maximumTrackedFaces;
    bool disableFaceChangeLock;
    cv::Mat reconstructionAverage;
    // A store for the homographies of faces that are being tracked
    cv::Mat currentHomographies[6];
    // Temporal image store
    cv::Mat reconstructionImages[6];
    // To do a full reconstruction only the first time, based on selected face
    bool firstIteration;
    int forceTrackIndex;

public:
    cv::Mat cuboidReconstructions[6];

    int calculateFaceWithLeastError();
    int calculateFaceWithNormalMostAligned();

    CuboidTrackerMultipleFace();
    void trackObject(cv::Mat &newImage);
    void highlightObject(cv::Mat &targetImage);
    void setMaximumTrackedFaces(int maximumTrackedFaces);
    void setFaceChangeLock(bool disable);
};

#endif // CUBOIDTRACKERMULTIPLEFACE_H
