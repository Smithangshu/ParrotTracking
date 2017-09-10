#ifndef CUBOID_TRACKER_H
#define CUBOID_TRACKER_H

#include "opencv2/core/core.hpp"
#include "tracking/Tracker.h"
#include "tracking/ESMTracker.h"
#include "tracking/ESMTrackerPyramidal.h"
#include "tracking/ESMTrackerGPU.h"
#include "tracking/MeanShiftTracker.h"
#include "reconstruction/HomographyDecomposer.h"

class CuboidTracker : public Tracker
{
	protected:
		void initFacesMappings();
		void calculateBaseTransformations();
		void calculateInverseBaseTransformations();

	public:
		int face;
		// Full transformations set. For easier access
		cv::Mat homographies[6];
		cv::Mat baseTransformations[6];
		cv::Mat baseInverseTransformations[6];
        cv::Mat baseToCenterTransform;
        cv::Mat centerToBaseTransform;
        cv::Mat centerTransform;
        cv::Mat centerTransformAlternate;
        cv::Mat centerTransformFixed;
        cv::Mat transformations[6];
		cv::Mat cameraIntrinsicParameters;
		// Store relative dimensions.
        cv::Size2i facesDimensions[6];
        cv::Size2f realFacesDimensions[6];
        int facesMappings[6][6];
		// Colors are attached to each face
		cv::Scalar faceColors[6];
		// Normals
		cv::Mat normals[6];
        // Face lock to force a given face to be tracked
        bool lockFace;
        // Boolean array which true elements are visible
		int faceIsVisible[6];
		// Trackers
        ESMTrackerPyramidal trackers[6];
        //ESMTrackerGPU trackers[6];
        MeanShiftTracker globalTracker;
		// For homography decomposition
		HomographyDecomposer homographyDecomposer;
		// Common transformation set. One could be shared across cubes.
		cv::Mat rotateX;
		cv::Mat rotateMinusX;
		cv::Mat rotateY;
		cv::Mat rotateMinusY;
		cv::Mat rotateZ;
		cv::Mat rotateMinusZ;
		cv::Mat temp_transformation_matrix;
		cv::Mat temp_transformation_reduced;
		cv::Mat temp_vector;
		cv::Mat temp_res_vector;
        // Use real world dimensions
        bool useRealWorldDimensions;
        cv::Size2f realWorldToTemplateScalings[6];
        // Cuboid tracking mode
        enum CuboidTrackingModes {SINGLE_VIEW, MULTI_VIEW} cuboidTrackingMode;
        enum CuboidTrackingOptimizations {NONE, FACE_WITH_LEAST_ERROR, MOST_ALIGNED_NORMAL} cuboidTrackingOptimization;

		CuboidTracker();
		void setBaseTracker(int face);
        void calculateTranslationFromGlobalTracker(cv::Mat &image);
		void calculateTransformationFromHomography();
        void calculateAllTransformationsFromSingleTransformation(bool useCenter = false);
        void calculateHomographiesFromTransformation(bool skipCurrentFace = true);
        void setModel(int face, cv::Mat &face0, cv::Mat &face1, cv::Mat &face2, cv::Mat &face3, cv::Mat &face4, cv::Mat &face5);
		static void reduceTransformationMatrix(cv::Mat &sourceMat, cv::Mat &destinationMat);
		void printCenterMatrix();
        void enableFaceLock(bool enableLock);

		void updateNormals();
		void calculateVisibleFaces();

		void getHomographyParameters(int face, cv::Mat &parameters);
        void setFacesDimensions(cv::Size2i face0, cv::Size2i face1, cv::Size2i face2);
        void setRealFacesDimensions(cv::Size2f face0, cv::Size2f face1, cv::Size2f face2);
        int mapFaceFromBase(int base, int face);

        ESMTracker* getTracker(int index);
};

#endif
