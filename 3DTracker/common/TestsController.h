#ifndef TESTS_CONTROLLER_H
#define TESTS_CONTROLLER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/core/core.hpp>
#include "tracking/CuboidTracker.h"

class TestsController 
{
	public:
        static enum TestTypes {ESM_TEST, ESM_PYR_TEST, ESM_COLOR_TEST, ESM_GPU_TEST, ESM_CONSTRAINED, CUBOID_TEST, MEAN_SHIFT_TEST, ESM_BENCHMARK, MEAN_SHIFT_SIMILARITY, PLANE_DETECTOR_TEST} testTypes;
        static enum VideoModes {CAPTURE_CAMERA, CAPTURE_VIDEO, CAPTURE_IMAGE} videoModes;
        static enum DecompositionModes {NO_DECOMPOSITION, SIMPLE_DECOMPOSITION, MALIS_DECOMPOSITION} decompositionModes;
        static enum DisplayModes {DISPLAY_2D, DISPLAY_OPENGL, DISPLAY_BOTH} displayModes;
        static enum SelectionModes {SELECTION_RECTANGLE, SELECTION_HOMOGRAPHY} selectionModes;
        std::string basePath;

		TestsController(const char *basePath);

		void loadTestsFromFile(const char *fileName);
        void loadModelsFromFile(const char *fileName);

        struct GenericTest
        {
            std::string testString;
            std::string id;
            enum TestTypes testType;
            enum VideoModes videoMode;
            std::string videoFileName;
            std::string imageFileName;
            std::string imageFileName2;
            cv::Mat images[2];
            int cameraIndex;
            enum DecompositionModes decompositionMode;
            enum DisplayModes displayMode;
            std::string templateFile;
            enum SelectionModes selectionMode;
            cv::Rect selectionRectangle;
            int forceFace;
            int topPyr;

            std::string texturesPath;
            cv::Mat textures[6];
            bool useRealDimensions;
            int cuboidWidth;
            int cuboidHeight;
            int cuboidDepth;
            cv::Rect globalTrackerSelectionRectangle;
            cv::Mat selectionPointsFront;
            cv::Mat selectionPointsTop;
            cv::Mat selectionPointsSide;
            int selectedFace;
            int enableMultiFace;
            int maxTrackedFaces;
            bool disableFaceChangeLock;
            enum CuboidTracker::CuboidTrackingModes cuboidTrackingMode;
            enum CuboidTracker::CuboidTrackingOptimizations cuboidTrackingOptimization;
        } tests[50], models[50];
        int totalTests;
        int totalModels;
        int currentTestIndex;

        void launchTest(int testIndex);
};


#endif
