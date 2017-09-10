#include "common/TestsController.h"
#include "common//TrackingCommon.h"
#include "tracking/ESMTrackerGPU.h"
#include "tracking/ESMTrackerPyramidal.h"
#include "tracking/ESMTrackerConstrained.h"
#include "tracking/ESMTrackerPyramidalFixed.h"
#include "tracking/MeanShiftTracker.h"
#include <QDebug>

TestsController::TestsController(const char *basePath)
{
    this->basePath = std::string(basePath);

    std::string configFile;
    configFile += basePath;
    configFile += std::string("Tests.txt");
    this->loadTestsFromFile(configFile.c_str());

    std::string modelsFile;
    modelsFile += basePath;
    modelsFile += std::string("CuboidModels.txt");
    this->loadModelsFromFile(modelsFile.c_str());
}

void TestsController::loadModelsFromFile(const char *fileName)
{
    std::ifstream file(fileName);
    std::string value;
    std::string nextCommand;

    while (std::getline(file, value))
    {
        GenericTest test;
        std::stringstream currentLine(value);

        std::string token;
        std::getline(currentLine, token, ',');
        test.id = token;
        test.testString = token;

        while (std::getline(currentLine, nextCommand, ','))
        {
            if (nextCommand == "TEXTURES_PATH")
            {
                std::getline(currentLine, token, ',');
                test.texturesPath = token;
            }
            else if (nextCommand == "TEXTURE_FACE")
            {
                int faceIndex = 0;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> faceIndex;

                std::stringstream fullTexturePath;
                std::getline(currentLine, token, ',');
                fullTexturePath << basePath << test.texturesPath << token;
                test.textures[faceIndex] = cv::imread(fullTexturePath.str());
            }
        }

        models[totalModels] = test;
        ++totalModels;
    }
}

void TestsController::loadTestsFromFile(const char *fileName)
{
	std::ifstream file(fileName);
    std::string value;
    std::string nextCommand;

    while (std::getline(file, value))
	{
		GenericTest test;
        test.useRealDimensions = false;
        std::stringstream currentLine(value);

        std::string token;
        std::getline(currentLine, token, ',');
        test.id = token;
        test.testString = token;
        test.forceFace = -1;
        test.enableMultiFace = 0;
        test.maxTrackedFaces = 1;
        test.disableFaceChangeLock = false;

        std::getline(currentLine, token, ',');
        if (token == "ESM")
            test.testType = ESM_TEST;
        else if (token == "ESM_GPU")
            test.testType = ESM_GPU_TEST;
        else if (token == "ESM_PYR")
            test.testType = ESM_PYR_TEST;
        else if (token == "ESM_COLOR_TEST")
            test.testType = ESM_COLOR_TEST;
        else if (token == "ESM_CONSTRAINED")
            test.testType = ESM_CONSTRAINED;
        else if (token == "CUBOID")
            test.testType = CUBOID_TEST;
        else if (token == "ESM_BENCHMARK")
            test.testType = ESM_BENCHMARK;
        else if (token == "MEAN_SHIFT_SIMILARITY")
            test.testType = MEAN_SHIFT_SIMILARITY;
        else if (token == "PLANE_DETECTOR_TEST")
            test.testType = PLANE_DETECTOR_TEST;

        while (std::getline(currentLine, nextCommand, ','))
        {
            if (nextCommand == "CAPTURE_CAMERA")
            {
                test.videoMode = CAPTURE_CAMERA;
                std::stringstream(token) >> test.cameraIndex;
            }
            else if (nextCommand == "CAPTURE_VIDEO")
            {
                test.videoMode = CAPTURE_VIDEO;
                std::getline(currentLine, token, ',');
                test.videoFileName = token;
            }
            else if (nextCommand == "CAPTURE_IMAGE" || nextCommand == "CAPTURE_IMAGE_2")
            {
                test.videoMode = CAPTURE_IMAGE;
                std::getline(currentLine, token, ',');

                std::stringstream fullImagePath;
                fullImagePath << basePath << token;

                if (nextCommand == "CAPTURE_IMAGE")
                {
                    test.imageFileName = token;
                    test.images[0] = cv::imread(fullImagePath.str());
                }
                else
                {
                    test.imageFileName2 = token;
                    test.images[1] = cv::imread(fullImagePath.str());
                }
            }
            else if (nextCommand == "NO_DECOMPOSITION")
            {
                test.decompositionMode = NO_DECOMPOSITION;
            }
            else if (nextCommand == "MALIS_DECOMPOSITION")
            {
                test.decompositionMode = MALIS_DECOMPOSITION;
            }
            else if (nextCommand == "SIMPLE_DECOMPOSITION")
            {
                test.decompositionMode = SIMPLE_DECOMPOSITION;
            }
            else if (nextCommand == "DISPLAY_2D")
            {
                test.displayMode = DISPLAY_2D;
            }
            else if (nextCommand == "DISPLAY_OPENGL")
            {
                test.displayMode = DISPLAY_OPENGL;
            }
            else if (nextCommand == "DISPLAY_BOTH")
            {
                test.displayMode = DISPLAY_BOTH;
            }
            else if (nextCommand == "SELECTION_RECTANGLE")
            {
                test.selectionMode = SELECTION_RECTANGLE;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.selectionRectangle.x;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.selectionRectangle.y;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.selectionRectangle.width;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.selectionRectangle.height;
            }
            else if (nextCommand == "SELECTION_HOMOGRAPHY")
            {
                test.selectionMode = SELECTION_HOMOGRAPHY;
                test.selectionPointsFront = cv::Mat(4, 2, CV_32F);
                for (int i = 0; i < 4; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        std::getline(currentLine, token, ',');
                        std::stringstream(token) >> test.selectionPointsFront.at<float>(i, j);
                    }
                }
            }
            else if (nextCommand == "GLOBAL_TRACKER")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.globalTrackerSelectionRectangle.x;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.globalTrackerSelectionRectangle.y;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.globalTrackerSelectionRectangle.width;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.globalTrackerSelectionRectangle.height;
            }
            else if (nextCommand == "VIEW_MODE")
            {
                std::getline(currentLine, token, ',');
                if (token == "MULTI_VIEW")
                    test.cuboidTrackingMode = CuboidTracker::MULTI_VIEW;
                else
                    test.cuboidTrackingMode = CuboidTracker::SINGLE_VIEW;
            }
            else if (nextCommand == "FACE_SELECTION_METHOD")
            {
                std::getline(currentLine, token, ',');
                if (token == "MOST_ALIGNED_NORMAL")
                    test.cuboidTrackingOptimization = CuboidTracker::MOST_ALIGNED_NORMAL;
                else
                    test.cuboidTrackingOptimization = CuboidTracker::FACE_WITH_LEAST_ERROR;
            }
            else if (nextCommand == "TEXTURES_PATH")
            {
                std::getline(currentLine, token, ',');
                test.texturesPath = token;
            }
            else if (nextCommand == "TEXTURE_FACE")
            {
                int faceIndex = 0;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> faceIndex;

                std::stringstream fullTexturePath;
                std::getline(currentLine, token, ',');
                fullTexturePath << basePath << test.texturesPath << token;
                test.textures[faceIndex] = cv::imread(fullTexturePath.str());
            }
            else if (nextCommand == "SELECTED_FACE")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.selectedFace;
            }
            else if (nextCommand == "TOP_PYR")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.topPyr;
            }
            else if (nextCommand == "FORCE_FACE")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.forceFace;
            }
            else if (nextCommand == "REAL_WORLD_DIMENSIONS")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.cuboidWidth;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.cuboidHeight;
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.cuboidDepth;
                test.useRealDimensions = true;
            }
            else if (nextCommand == "MULTIFACE_TRACKING")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.enableMultiFace;
            }
            else if (nextCommand == "MAX_TRACKED_FACES")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.maxTrackedFaces;
            }
            else if (nextCommand == "DISABLE_FACE_CHANGE_LOCK")
            {
                std::getline(currentLine, token, ',');
                std::stringstream(token) >> test.disableFaceChangeLock;
            }
            else if (nextCommand == "COMMAND")
            {
            }
        }

        tests[totalTests] = test;
        ++totalTests;
    }
}

void TestsController::launchTest(int testIndex)
{
    currentTestIndex = testIndex;
    GenericTest test = tests[testIndex];
    if (test.testType == TestsController::MEAN_SHIFT_TEST)
    {
    }
    else if (test.testType == TestsController::ESM_TEST || test.testType == TestsController::ESM_GPU_TEST || test.testType == TestsController::ESM_PYR_TEST || test.testType == TestsController::ESM_COLOR_TEST || test.testType == TestsController::ESM_CONSTRAINED)
    {
        qDebug() << "Creating ESM Test";
        ESMTracker* tracker;
        if (test.testType == TestsController::ESM_TEST)
            tracker = new ESMTracker();
        else if (test.testType == TestsController::ESM_PYR_TEST || test.testType == TestsController::ESM_COLOR_TEST)
        {
            tracker = new ESMTrackerPyramidal();
            tracker->setTopPyramid(test.topPyr);
            if (test.testType == TestsController::ESM_COLOR_TEST)
            {
                tracker->enableColor(true);
                tracker->setTopPyramid(0);
                qDebug() << "Color Enabled";
            }
        }
        else if (test.testType == TestsController::ESM_CONSTRAINED)
        {
            tracker = new ESMTrackerConstrained();
            tracker->enableColor(false);
            tracker->setTopPyramid(0);
        }
        else
            tracker = new ESMTrackerGPU();

        if (test.selectionMode == SELECTION_HOMOGRAPHY) {
            qDebug() << "Showing UNWARPED";
            cv::Mat sourcePlanePoints = cv::Mat(4, 2, CV_32F);
            cv::Mat destinationPlanePoints = test.selectionPointsFront;
            cv::Mat homography;
            cv::Mat sourceImage = TrackingCommon::frame.clone();
            cv::Mat unwarpedImage = TrackingCommon::frame.clone();
            cv::Mat model;
            cv::Rect modelSelection;

            sourcePlanePoints.at<float>(0, 0) = 0;
            sourcePlanePoints.at<float>(0, 1) = 0;
            sourcePlanePoints.at<float>(1, 0) = 150;
            sourcePlanePoints.at<float>(1, 1) = 0;
            sourcePlanePoints.at<float>(2, 0) = 150;
            sourcePlanePoints.at<float>(2, 1) = 150;
            sourcePlanePoints.at<float>(3, 0) = 0;
            sourcePlanePoints.at<float>(3, 1) = 150;

            homography = cv::findHomography(sourcePlanePoints, destinationPlanePoints);

            cv::warpPerspective(sourceImage, unwarpedImage, homography, unwarpedImage.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP);
            //cv::imshow("Unwarped", unwarpedImage);

            tracker->setTargetObject(unwarpedImage, cv::Rect(0, 0, 150, 150));
            tracker->setHomography(homography);

        } else {
            tracker->setTargetObject(TrackingCommon::frame, test.selectionRectangle);
        }
        // Make sure we do a SetZero
        TrackingCommon::markObjectZero = true;

        TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = tracker;
        if (test.displayMode != TestsController::DISPLAY_2D)
        {
            PlaneDrawing* planeDrawing = new PlaneDrawing();
            TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D] = planeDrawing;
            planeDrawing->setTracker(tracker);
            cv::Mat selectedTarget = tracker->targetObject.clone();
            planeDrawing->setGLParameters(Drawing3D::MULTI_VIEW, "3D Reconstruction", selectedTarget);

            ++TrackingCommon::totalDrawing3D;
            TrackingCommon::mainController->enable3DShowReconstruction(true);
            TrackingCommon::show3DReconstruction = true;
        }
        ++TrackingCommon::totalWindows;
    }
    else if (test.testType == TestsController::CUBOID_TEST)
    {
        qDebug() << "Cuboid Test";

        TrackingCommon::mainController->setMultiFaceTracking(test.enableMultiFace);

        TrackingCommon::trackingMode = TrackingCommon::TRACKING_CUBOID;
        CuboidDrawing* cuboid = new CuboidDrawing();
        TrackingCommon::drawings3D[TrackingCommon::totalDrawing3D] = cuboid;
        cuboid->setDefaultCamera();

        CuboidTracker* cuboidTracker = cuboid->getCuboidTracker();
        TrackingCommon::trackingWindows[TrackingCommon::totalWindows] = cuboidTracker;
        ++TrackingCommon::totalWindows;
        ++TrackingCommon::totalDrawing3D;
        if (test.displayMode != TestsController::DISPLAY_2D)
        {
            TrackingCommon::mainController->enable3DShowReconstruction(true);
            TrackingCommon::show3DReconstruction = true;
        }

        //cuboidTracker->globalTracker.setTargetObject(TrackingCommon::currentFrame, test.globalTrackerSelectionRectangle);

        cuboidTracker->cuboidTrackingMode = test.cuboidTrackingMode;
        cuboidTracker->cuboidTrackingOptimization = test.cuboidTrackingOptimization;

        cv::Mat sourcePlanePoints = cv::Mat(4, 2, CV_32F);
        cv::Mat destinationPlanePoints = test.selectionPointsFront;
        cv::Mat homography;

        sourcePlanePoints.at<float>(0, 0) = 0;
        sourcePlanePoints.at<float>(0, 1) = 0;
        sourcePlanePoints.at<float>(1, 0) = test.textures[test.selectedFace].cols;
        sourcePlanePoints.at<float>(1, 1) = 0;
        sourcePlanePoints.at<float>(2, 0) = test.textures[test.selectedFace].cols;
        sourcePlanePoints.at<float>(2, 1) = test.textures[test.selectedFace].rows;
        sourcePlanePoints.at<float>(3, 0) = 0;
        sourcePlanePoints.at<float>(3, 1) = test.textures[test.selectedFace].rows;

        homography = cv::findHomography(sourcePlanePoints, destinationPlanePoints);

        cuboid->prepareTextures(test.textures[0], test.textures[1], test.textures[2], test.textures[3], test.textures[4], test.textures[5]);
        cuboidTracker->setFacesDimensions(cv::Size2i(test.textures[0].cols, test.textures[0].rows), cv::Size2i(test.textures[1].cols, test.textures[1].rows), cv::Size2i(test.textures[4].cols, test.textures[4].rows));
        if (test.useRealDimensions) {
            cuboidTracker->setRealFacesDimensions(cv::Size2f(test.cuboidWidth, test.cuboidHeight), cv::Size2i(test.cuboidDepth, test.cuboidWidth), cv::Size2i(test.cuboidDepth, test.cuboidHeight));
        }
        cuboidTracker->setModel(test.selectedFace, test.textures[0], test.textures[1], test.textures[2], test.textures[3], test.textures[4], test.textures[5]);

        // Set the initial homography
        qDebug() << "test.selectedFace " << test.selectedFace;
        cuboidTracker->setBaseTracker(test.selectedFace);
        cuboidTracker->trackers[test.selectedFace].setHomography(homography);

        if (test.maxTrackedFaces)
        {
            qDebug() << "Max Tracked Faces: " << test.maxTrackedFaces;
            ((CuboidTrackerMultipleFace*) cuboidTracker)->setMaximumTrackedFaces(test.maxTrackedFaces);
        }
        // Enable or disable the limit on the speed of face change lock
        ((CuboidTrackerMultipleFace*) cuboidTracker)->setFaceChangeLock(test.disableFaceChangeLock);

        // Calculate initial cuboid state
        cuboidTracker->enableFaceLock(test.forceFace >= 0);
        cuboidTracker->calculateTransformationFromHomography();
        cuboidTracker->calculateAllTransformationsFromSingleTransformation();
        cuboidTracker->calculateHomographiesFromTransformation();

        cuboid->setGLParameters(Drawing3D::MULTI_VIEW, "Cuboid 3D Reconstruction");
    }
    else if (test.testType == TestsController::ESM_BENCHMARK)
    {
        qDebug() << "Starting ESM Benchmark";

        std::stringstream fullImagePath;
        fullImagePath << basePath << test.imageFileName;

        cv::Mat testImage = cv::imread(fullImagePath.str().c_str());
        cv::Mat testParameters = cvCreateMat(8, 1, CV_32F);

        float scales[4];
        scales[0] = 0.5; scales[1] = 0.75; scales[2] = 1.0; scales[3] = 1.25; scales[4] = 1.5;

        float tests_params[4][2];
        tests_params[0][1] = 0.00025; tests_params[0][2] = 4.0;
        tests_params[1][1] = 0.00050; tests_params[1][2] = 6.0;
        tests_params[2][1] = 0.00075; tests_params[2][2] = 10.0;
        tests_params[3][1] = 0.00100; tests_params[3][2] = 13.0;

        for (int p = 0; p < 1; ++p) // Pyramids
        {
            for (int mask = 0; mask < 1; ++mask) // Masks
            {
                for (int color = 0; color < 2; ++color) // Color
                {
                    for (int m = 0; m < 5; ++m) // Scales
                    {
                        cv::Rect testWindow = cv::Rect(testImage.cols / 2 - scales[m] * 75, testImage.rows / 2 - scales[m] * 75, scales[m] * 150, scales[m] * 150);
                        ESMTracker* tracker = new ESMTrackerPyramidal();

                        tracker->enableColor(color);
                        tracker->setTargetObject(testImage, testWindow);
                        tracker->parameters.copyTo(testParameters);
                        tracker->setTopPyramid(p);
                        tracker->enableMaskedTemplate(mask);

                        for (int i = 0; i < 4; ++i) // Tests
                        {
                            int current_test = 0;
                            double test_parameters_values[8];
                            while (current_test++ < 100)
                            {
                                qDebug() << "Test Pyr: " << p << " Mask:" << mask << " Color: " << color << " Scale: " << m << " Test:" << i << " " << current_test;
                                for (int j = 0; j < 6; ++j) {
                                    test_parameters_values[j] = tests_params[i][1] * ((double) (rand() % 200 - 100)) / 100.0;
                                }
                                test_parameters_values[6] = testWindow.x + rand() % (int) (scales[m] * tests_params[i][2]) - (int) (scales[m] * tests_params[i][2]) / 2;
                                test_parameters_values[7] = testWindow.y + rand() % (int) (scales[m] * tests_params[i][2]) - (int) (scales[m] * tests_params[i][2]) / 2;

                                SET_PARAMETERS(tracker->parameters, test_parameters_values[0], test_parameters_values[1], test_parameters_values[2], test_parameters_values[3], test_parameters_values[4], test_parameters_values[5], test_parameters_values[6], test_parameters_values[7]);
                                SET_TRANSFORM_MAT(tracker->W_3x3, tracker->parameters);

                                cv::Mat initial_image = testImage.clone();
                                tracker->drawWarpedRect(initial_image);
                                cv::imshow("InitialWarp", initial_image);

                                tracker->currentIteration = current_test - 1;
                                tracker->trackObject(testImage);

                                initial_image = testImage.clone();
                                tracker->drawWarpedRect(initial_image);
                                cv::imshow("EndWarp", initial_image);

                                cv::Mat results = cv::Mat(1, 9, CV_32F);
                                results.at<float>(0, 0) = p;
                                results.at<float>(0, 1) = mask;
                                results.at<float>(0, 2) = color;
                                results.at<float>(0, 3) = m;
                                results.at<float>(0, 4) = i;
                                results.at<float>(0, 5) = tracker->currentIteration;
                                results.at<float>(0, 6) = tracker->currentSubIteration;
                                results.at<float>(0, 7) = tracker->lastError;
                                results.at<float>(0, 8) = tracker->lastTime;
                                TrackingCommon::mainController->displayData("Benchmark", results, DataDisplayer::APPEND);
                                cv::waitKey(30);
                            }
                        }
                    }
                }
            }
        }
        qDebug() << "\r\nTests Ended";
        cv::destroyAllWindows();

    }
    else if (test.testType == TestsController::MEAN_SHIFT_SIMILARITY)
    {
        std::stringstream fullImagePath;
        fullImagePath << basePath << test.imageFileName;

        cv::Mat testImage = cv::imread(fullImagePath.str().c_str());

        MeanShiftTracker* tracker = new MeanShiftTracker();
        tracker->setTargetObject(testImage, test.selectionRectangle);

        double pu_i[MeanShiftTracker::colorBins][MeanShiftTracker::colorBins][MeanShiftTracker::colorBins];
        double temp;
        cv::Mat Img_Target = cv::Mat(testImage.rows, testImage.cols, CV_8UC3);
        cv::Mat Img_Target_b = cv::Mat(testImage.rows, testImage.cols, CV_8UC3);
        cv::Mat Img_Target_c = cv::Mat(testImage.rows, testImage.cols, CV_8UC3);

        Img_Target.setTo(0);
        Img_Target_b.setTo(0);
        Img_Target_c.setTo(0);

        for (int y = tracker->trackingWindow.height / 2; y < testImage.rows - tracker->trackingWindow.height / 2; y++ ) {
            uchar* prt = Img_Target.row(y).ptr();
            uchar* prt_2 = Img_Target_b.row(y).ptr();
            uchar* prt_3 = Img_Target_c.row(y).ptr();

            for (int x = tracker->trackingWindow.width / 2; x < testImage.cols - tracker->trackingWindow.width / 2; x++ ) {

                tracker->getPu(testImage, pu_i, cvPoint(x, y));
                temp = tracker->getBhattaCoefficient(tracker->qu, pu_i);

                int k = (int) (temp * 0xffffff);
                qDebug() << "Testing at " << y << "," << x << ": " << temp << ", " << k;
                if (k <= 0x444444) {
                    prt[x * 3] = (255 * k) / 0x444444;
                    prt[x * 3 + 1] = 0;
                    prt[x * 3 + 2] = 0;
                } else if (k <= 0x888888) {
                    prt[x * 3 + 1] = (255 * (k - 0x444444)) / 0x444444;
                    prt[x * 3] = 0xff - prt[x * 3 + 1];
                    prt[x * 3 + 2] = 0;
                } else if (k <= 0xCCCCCC) {
                    prt[x * 3] = 0;
                    prt[x * 3 + 2] = (255 * (k - 0x888888)) / 0x444444;
                    prt[x * 3 + 1] = 0xff - prt[x * 3 + 2];
                } else {
                    prt[x * 3] = 0;
                    prt[x * 3 + 1] = 0;
                    prt[x * 3 + 2] = 0xff - (255 * (k - 0xCCCCCC)) / 0x444444;
                }

                if (k <= 0x333333) {
                    prt_2[x * 3] = (255 * k) / 0x333333;
                    prt_2[x * 3 + 1] = 0;
                    prt_2[x * 3 + 2] = 0;
                } else if (k <= 0x666666) {
                    prt_2[x * 3] = 0xff;
                    prt_2[x * 3 + 1] = (255 * (k - 0x333333)) / 0x333333;
                    prt_2[x * 3 + 2] = 0;
                } else if (k <= 0x999999) {
                    prt_2[x * 3] = 0xff - (255 * (k - 0x666666)) / 0x333333;
                    prt_2[x * 3 + 1] = 0xff;
                    prt_2[x * 3 + 2] = (255 * (k - 0x666666)) / 0x333333;
                } else if (k <= 0xCCCCCC) {
                    prt_2[x * 3] = 0;
                    prt_2[x * 3 + 1] = 0xff - (255 * (k - 0x999999)) / 0x333333;
                    prt_2[x * 3 + 2] = 0xff;
                } else {
                    prt_2[x * 3] = 0;
                    prt_2[x * 3 + 1] = 0;
                    prt_2[x * 3 + 2] = 0xff - (255 * (k - 0xCCCCCC)) / 0x333333;
                }

                k = ( (int) temp ) * 255;
                prt_3[x * 3] = k & 0x0000ff;
                prt_3[x * 3 + 1] = (k >> 8) & 0x0000ff;
                prt_3[x * 3 + 2] = (k >> 16) & 0x0000ff;

                k = ( (int) temp ) * 65535;
            }
        }

        cv::imshow("Img", Img_Target);
    }
    else if (test.testType == TestsController::PLANE_DETECTOR_TEST)
    {
        qDebug() << "Plane detector test";
        cv::Mat srcImg = test.images[0];
        cv::Mat dstImg = test.images[1];
        cv::imshow("srcImg", srcImg);

        PlaneDetector detector;
        detector.addImage(srcImg);
        detector.addImage(dstImg);
        detector.findPlanesFlowHomography(dstImg);
        cv::imshow("dstImg", dstImg);
    }
}
