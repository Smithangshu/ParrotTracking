#include "gui/WindowCallbacks.h"
#include "gui/HomographyCaptureWindow.h"
#include "common/TrackingCommon.h"
#include <QDebug>

void WindowCallBack::mouseMove(int x, int y)
{

}

void WindowCallBack::mousePress(int x, int y)
{

}

void WindowCallBack::mouseRelease(int x, int y)
{

}

void TrackingCallback::mouseMove(int x, int y)
{
    TrackingCommon::mouseX = x;
    TrackingCommon::mouseY = y;
    if (TrackingCommon::programMode == TrackingCommon::TRACKING)
    {
        if (TrackingCommon::drawingBox)
        {
            TrackingCommon::selectionBox.width = x - TrackingCommon::selectionBox.x;
            TrackingCommon::selectionBox.height = y - TrackingCommon::selectionBox.y;
        }
        if (TrackingCommon::positionEstimatorController.isReady())
        {
            cv::Mat displayData = TrackingCommon::calculate2DPosition(cv::Point2f(x, y));
            TrackingCommon::dataDisplayer.displayCoordinates("Coordinates", "Mouse", displayData);
        }
    }
    else if (TrackingCommon::programMode == TrackingCommon::HOMOGRAPHY_DISCOVERY && TrackingCommon::zoomActive)
    {
        TrackingCommon::zoomImage(x, y);
    }
}

void TrackingCallback::mousePress(int x, int y)
{
    TrackingCommon::mouseX = x;
    TrackingCommon::mouseY = y;
    if (TrackingCommon::programMode == TrackingCommon::TRACKING)
    {
        TrackingCommon::selectionBox = cv::Rect(x, y, 0, 0);
        TrackingCommon::drawingBox = true;
    }
    else if (TrackingCommon::programMode == TrackingCommon::HOMOGRAPHY_DISCOVERY)
    {
        TrackingCommon::zoomActive = true;
        TrackingCommon::homographyCaptureWindow->show();
    }
}

void TrackingCallback::mouseRelease(int x, int y)
{
    TrackingCommon::mouseX = x;
    TrackingCommon::mouseY = y;
    if (TrackingCommon::programMode == TrackingCommon::TRACKING)
    {
        TrackingCommon::drawingBox = false;
        if (TrackingCommon::selectionBox.width < 0)
        {
            TrackingCommon::selectionBox.x += TrackingCommon::selectionBox.width;
            TrackingCommon::selectionBox.width *= -1;
        }
        if (TrackingCommon::selectionBox.height < 0)
        {
            TrackingCommon::selectionBox.y += TrackingCommon::selectionBox.height;
            TrackingCommon::selectionBox.height *= -1;
        }

        if (TrackingCommon::selectionBox.width < 3 || TrackingCommon::selectionBox.height < 3)
        {
                    // Start Homography Discovery Mode
                    switch (TrackingCommon::homographyDiscoveryStep)
                        {
                        case 0: {
                            TrackingCommon::drawingHomographyDiscovery = true;
                            qDebug() << "You should have selected Top left corner. Order is: TL, TR, BR, BL.\r\n";
                            TrackingCommon::homographyDiscoveryPoints[0] = cv::Point2f(x, y);
                            qDebug() << "\r\n (" << TrackingCommon::homographyDiscoveryPoints[0].x << "," << TrackingCommon::homographyDiscoveryPoints[0].y <<  ")\r\n";
                            qDebug() << "Please select three points more...";
                            TrackingCommon::homographyDiscoveryStep = 1;
                            break;
                        }
                        case 1:
                        case 2:
                        case 3: {
                            TrackingCommon::homographyDiscoveryPoints[TrackingCommon::homographyDiscoveryStep] = cv::Point2f(x, y);
                            qDebug() << " (" << TrackingCommon::homographyDiscoveryPoints[TrackingCommon::homographyDiscoveryStep].x << "," << TrackingCommon::homographyDiscoveryPoints[TrackingCommon::homographyDiscoveryStep].y;
                            TrackingCommon::homographyDiscoveryStep++;
                            if (TrackingCommon::homographyDiscoveryStep > 3) {
                                float template_width, template_height;
                                if (TrackingCommon::trackingMode == TrackingCommon::TRACKING_CUBOID)
                                {
                                    // Read image corresponding to selected face
                                    //cv::Mat selectedFace = cv::imread();
                                    TestsController::GenericTest test = TrackingCommon::testsController.models[TrackingCommon::cuboidSelectedModel];
                                    test.selectedFace = TrackingCommon::cuboidSelectedFace;
                                    template_width = test.textures[test.selectedFace].cols;
                                    template_height = test.textures[test.selectedFace].rows;
                                }
                                else
                                {
                                    // Allow user providing dimensions of this
                                }
                                cv::Point2f homography_plane_vertices[4];
                                homography_plane_vertices[0] = cv::Point2f(0, 0);
                                homography_plane_vertices[1] = cv::Point2f(template_width, 0);
                                homography_plane_vertices[2] = cv::Point2f(template_width, template_height);
                                homography_plane_vertices[3] = cv::Point2f(0, template_height);

                                cv::Mat image_points = cv::Mat(1, 4, CV_32FC2);
                                cv::Mat projected_points = cv::Mat(1, 4, CV_32FC2);
                                for (int i = 0; i < 4; ++i)
                                {
                                    image_points.at<cv::Point2f>(0, i) = homography_plane_vertices[i];
                                    projected_points.at<cv::Point2f>(0, i) = TrackingCommon::homographyDiscoveryPoints[i];
                                }

                                TrackingCommon::discoveredHomography = cv::findHomography(image_points, projected_points);
    //                            tracking_windows[total_windows] = new Tracking(frame, cv::Rect(0, 0, template_width, template_height), MALIS_ESM, false);
    //                            for (int i = 0; i < 3; ++i)
    //                                for (int j = 0; j < 2 || (j < 3 && i < 2); ++j)
    //                                    CV_MAT_ELEM(*(tracking_windows[total_windows]->parameters), double, 3 * i + j, 0) = CV_MAT_ELEM(*discovered_H, double, j, i);
    //                            CV_MAT_ELEM(*(tracking_windows[total_windows]->parameters), double, 0, 0) -= 1;
    //                            CV_MAT_ELEM(*(tracking_windows[total_windows]->parameters), double, 4, 0) -= 1;
    //                            tracking_windows[total_windows]->target_object = ref_frame;
    //                            tracking_windows[total_windows]->initializeLKTracking(frame);
    //                            ++total_windows;
                                TrackingCommon::homographyDiscoveryStep = 0;
                                TrackingCommon::createTrackingWindow = true;
                            }
                            break;
                        }
                    }
                    return;
        }
        else
        {
            if (TrackingCommon::trackingMode == TrackingCommon::TRACKING_MS || TrackingCommon::trackingMode == TrackingCommon::TRACKING_ESM)
            {
                TrackingCommon::createTrackingWindow = true;
            }
            else if (false)
            {
//                cuboidDrawings[totalCuboidDrawing] = CuboidDrawing();
//                cuboidDrawings[totalCuboidDrawing].createView(Drawing3D::MULTI_VIEW, "Cuboid");
//                CuboidTracker* cuboidTracker = cuboidDrawings[totalCuboidDrawing].getCuboidTracker();
//                ESMTracker* tracker = cuboidTracker->getTracker(0);
//                tracker->setTargetObject(frame, selectionBox);
//                cuboidTracker->setBaseTracker(0);
//                cuboidTracker->setFacesDimensions(cv::Size2i(selectionBox.width, selectionBox.height), cv::Size2i(selectionBox.width, selectionBox.height), cv::Size2i(selectionBox.width, selectionBox.height));
//                cv::Mat temp = tracker->targetObject.clone();
//                cuboidDrawings[totalCuboidDrawing].setTextures(temp);
//                ++totalCuboidDrawing;
            }
        }
    }
    else if (TrackingCommon::programMode == TrackingCommon::HOMOGRAPHY_DISCOVERY)
    {
        TrackingCommon::zoomActive = false;
        float interpolatedX = TrackingCommon::currentSelectionWindow.x;
        float interpolatedY = TrackingCommon::currentSelectionWindow.y;
        interpolatedX += (1.0 * TrackingCommon::currentSelectionWindow.width * x) / (TrackingCommon::currentFrame.cols);
        interpolatedY += (1.0 * TrackingCommon::currentSelectionWindow.height * y) / (TrackingCommon::currentFrame.rows);
        TrackingCommon::homographyCaptureWindow->addImagePoint((float)interpolatedX, (float)interpolatedY);
        // This is will be overriden later by homographyWindow
        TrackingCommon::positionEstimatorController.positionEstimator.addTrainingPoint(cv::Point2f((float)interpolatedX, (float)interpolatedY), cv::Point2f());
    }

}
