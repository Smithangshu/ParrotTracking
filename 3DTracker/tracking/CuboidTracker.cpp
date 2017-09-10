#include "tracking/CuboidTracker.h"
#include "common/TrackingCommon.h"
#include <iostream>
#include <fstream>

CuboidTracker::CuboidTracker()
{
    type = TYPE_CUBOID;
	cameraIntrinsicParameters = cv::Mat(3, 3, CV_64F);
	baseToCenterTransform = cv::Mat(4, 4, CV_64F);
	centerTransform = cv::Mat(4, 4, CV_64F);
	for (int i = 0; i < 6; ++i)
	{
        // Here we can set the max pyr desired
        trackers[i] = ESMTrackerPyramidal();
        //trackers[i] = ESMTrackerGPU();
        trackers[i].setTopPyramid(0);

		homographies[i] = cv::Mat(3, 3, CV_64F);
		cv::setIdentity(homographies[i]);
		transformations[i] = cv::Mat(4, 4, CV_64F);
		cv::setIdentity(transformations[i]);
		baseTransformations[i] = cv::Mat(4, 4, CV_64F);
		cv::setIdentity(baseTransformations[i]);
		baseInverseTransformations[i] = cv::Mat(4, 4, CV_64F);
		cv::setIdentity(baseInverseTransformations[i]);
		normals[i] = cv::Mat::zeros(4, 1, CV_64F);
	}
	globalTracker = MeanShiftTracker();
	homographyDecomposer = HomographyDecomposer();

    assert(("Calibration has not been loaded", TrackingCommon::calibrated));

	homographyDecomposer.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);

	cuboidTrackingMode = SINGLE_VIEW;
	cuboidTrackingOptimization = NONE;
    useRealWorldDimensions = false;


	temp_transformation_matrix = cv::Mat(4, 4, CV_64F);
	temp_transformation_reduced = cv::Mat(3, 3, CV_64F);
	temp_vector = cv::Mat(4, 1, CV_64F);
	temp_res_vector = cv::Mat(4, 1, CV_64F);
	rotateX = cv::Mat(4, 4, CV_64F);
	rotateMinusX = cv::Mat(4, 4, CV_64F);
	rotateY = cv::Mat(4, 4, CV_64F);
	rotateMinusY = cv::Mat(4, 4, CV_64F);
	rotateZ = cv::Mat(4, 4, CV_64F);
	rotateMinusZ = cv::Mat(4, 4, CV_64F);
	cv::setIdentity(rotateX);
	rotateX.at<double>(1, 1) = 0;
	rotateX.at<double>(2, 1) = 1;
	rotateX.at<double>(1, 2) = -1;
	rotateX.at<double>(2, 2) = 0;
	cv::setIdentity(rotateMinusX);
	rotateMinusX.at<double>(1, 1) = 0;
	rotateMinusX.at<double>(2, 1) = -1;
	rotateMinusX.at<double>(1, 2) = 1;
	rotateMinusX.at<double>(2, 2) = 0;
	cv::setIdentity(rotateY);
	rotateY.at<double>(0, 0) = 0;
	rotateY.at<double>(2, 0) = -1;
	rotateY.at<double>(0, 2) = 1;
	rotateY.at<double>(2, 2) = 0;
	cv::setIdentity(rotateMinusY);
	rotateMinusY.at<double>(0, 0) = 0;
	rotateMinusY.at<double>(2, 0) = 1;
	rotateMinusY.at<double>(0, 2) = -1;
	rotateMinusY.at<double>(2, 2) = 0;
	cv::setIdentity(rotateZ);
	rotateZ.at<double>(0, 0) = 0;
	rotateZ.at<double>(1, 0) = 1;
	rotateZ.at<double>(0, 1) = -1;
	rotateZ.at<double>(1, 1) = 0;
	cv::setIdentity(rotateMinusZ);
	rotateMinusZ.at<double>(0, 0) = 0;
	rotateMinusZ.at<double>(1, 0) = -1;
	rotateMinusZ.at<double>(0, 1) = 1;
	rotateMinusZ.at<double>(1, 1) = 0;

	initFacesMappings();

	face = 0;
	faceColors[0] = cv::Scalar(255, 0, 0);
	faceColors[1] = cv::Scalar(0, 255, 0);
	faceColors[2] = cv::Scalar(128, 0, 0);
	faceColors[3] = cv::Scalar(0, 128, 0);
	faceColors[4] = cv::Scalar(0, 0, 255);
	faceColors[5] = cv::Scalar(0, 0, 128);
	
	setFacesDimensions(cv::Size2i(150, 100), cv::Size2i(150, 200), cv::Size2i(200, 100));

    // Kalman filter
    filter.initialize(6);
}

void CuboidTracker::setBaseTracker(int face)
{
	this->face = face;
}

void CuboidTracker::calculateTranslationFromGlobalTracker(cv::Mat &image)
{
	cv::Point previousTranslation = cv::Point(this->globalTracker.trackingWindow.x, this->globalTracker.trackingWindow.y);
	this->globalTracker.trackObject(image);
	cv::Point newTranslation = cv::Point(this->globalTracker.trackingWindow.x, this->globalTracker.trackingWindow.y);
	cv::Point deltaTranslation = cv::Point(newTranslation.x - previousTranslation.x, newTranslation.y - previousTranslation.y);

    cv::Mat currentHomography = trackers[face].W_3x3.clone();
	currentHomography.at<double>(0, 2) += deltaTranslation.x;
	currentHomography.at<double>(1, 2) += deltaTranslation.y;
	trackers[face].setHomography(currentHomography);
}

void CuboidTracker::calculateTransformationFromHomography()
{
    cv::Mat newHomography;
    trackers[face].W_3x3.convertTo(newHomography, CV_64F);
    //trackers[face].W_3x3.copyTo(homographies[face]);
    newHomography.copyTo(homographies[face]);
    homographyDecomposer.setHomography(homographies[face]);
	homographyDecomposer.decompose();
    homographyDecomposer.transform.copyTo(transformations[face]);
}

void CuboidTracker::calculateAllTransformationsFromSingleTransformation(bool useCenter)
{
	cv::Mat virtualTransform = transformations[face].clone();

    if (!useCenter)
    {
        // Go to base transform (at face 0)
        cv::gemm(transformations[face], baseInverseTransformations[face], 1.0, cv::Mat(), 0, virtualTransform);

        // Transformation to obtain cuboid center
        cv::gemm(virtualTransform, baseToCenterTransform, 1.0, cv::Mat(), 0, centerTransform);

    }
    else
    {
        cv::gemm(centerTransform, centerToBaseTransform, 1.0, cv::Mat(), 0, virtualTransform);
    }

    // Filter center
    if (TrackingCommon::enableFiltering3DPose) {
        qDebug() << "Current tracking is filtered";
        cv::Mat rotation = centerTransform(cv::Range(0, 3), cv::Range(0, 3));
        cv::Mat angles;
        cv::Rodrigues(rotation, angles);
        // Prepare input for kalman filter
        cv::Mat currentPose = cv::Mat(6, 1, CV_32F);
        currentPose.at<float>(0, 0) = angles.at<double>(0, 0);
        currentPose.at<float>(1, 0) = angles.at<double>(1, 0);
        currentPose.at<float>(2, 0) = angles.at<double>(2, 0);
        currentPose.at<float>(3, 0) = centerTransform.at<double>(3, 0);
        currentPose.at<float>(4, 0) = centerTransform.at<double>(3, 1);
        currentPose.at<float>(5, 0) = centerTransform.at<double>(3, 2);
        if (TrackingCommon::enableFiltering3DPoseToggled) {
            filter.setState(currentPose);
            TrackingCommon::enableFiltering3DPoseToggled = false;
        }
        filter.filter(currentPose);
        // Return data
        angles.at<double>(0, 0) = filter.FPose.at<float>(0, 0);
        angles.at<double>(1, 0) = filter.FPose.at<float>(1, 0);
        angles.at<double>(2, 0) = filter.FPose.at<float>(2, 0);
        cv::Rodrigues(angles, rotation);
        centerTransform.at<double>(3, 0) = filter.FPose.at<float>(3, 0);
        centerTransform.at<double>(3, 1) = filter.FPose.at<float>(4, 0);
        centerTransform.at<double>(3, 2) = filter.FPose.at<float>(5, 0);

        // Update virtual transform
        cv::gemm(centerTransform, centerToBaseTransform, 1.0, cv::Mat(), 0, virtualTransform);
    }

    for (int i = 0; i < 6; ++i)
	{
        cv::gemm(virtualTransform, baseTransformations[i], 1.0, cv::Mat(), 0, transformations[i]);
	}
}

void CuboidTracker::calculateHomographiesFromTransformation(bool skipCurrentFace)
{
	for (int i = 0; i < 6; ++i)
	{
        // Until calibration associated problems be solver, better omit updating current face
        if (i != face || !skipCurrentFace)
        {
            reduceTransformationMatrix(transformations[i], temp_transformation_reduced);
            cv::gemm(TrackingCommon::cameraMatrix, temp_transformation_reduced, 1, cv::Mat(), 0, homographies[i]);

            double multiplier = 1 / homographies[i].at<double>(2, 2);
            homographies[i].at<double>(0, 0) *= multiplier;
            homographies[i].at<double>(1, 0) *= multiplier;
            homographies[i].at<double>(2, 0) *= multiplier;
            homographies[i].at<double>(0, 1) *= multiplier;
            homographies[i].at<double>(1, 1) *= multiplier;
            homographies[i].at<double>(2, 1) *= multiplier;
            homographies[i].at<double>(0, 2) *= multiplier;
            homographies[i].at<double>(1, 2) *= multiplier;
            homographies[i].at<double>(2, 2) *= multiplier;

            // Update tracker homography
            trackers[i].setHomography(homographies[i]);
        }
	}
}

void CuboidTracker::printCenterMatrix()
{
	fstream debugFile; debugFile.open("CuboidCenter.txt", ios::app);	
	debugFile << "X," << centerTransform.at<double>(0, 0) << "," << centerTransform.at<double>(1, 0) << "," << centerTransform.at<double>(2, 0) << ",";
	debugFile << "Y," << centerTransform.at<double>(0, 1) << "," << centerTransform.at<double>(1, 1) << "," << centerTransform.at<double>(2, 1) << ",";
	debugFile << "Z," << centerTransform.at<double>(0, 2) << "," << centerTransform.at<double>(1, 2) << "," << centerTransform.at<double>(2, 2) << ",";
	debugFile << "T," << centerTransform.at<double>(0, 3) << "," << centerTransform.at<double>(1, 3) << "," << centerTransform.at<double>(2, 3) << ",";
	debugFile << endl;
	debugFile.close();
}

void CuboidTracker::enableFaceLock(bool enableLock)
{
    lockFace = enableLock;
}

void CuboidTracker::updateNormals()
{
	temp_vector.at<double>(0, 0) = 0;
	temp_vector.at<double>(1, 0) = 0;
	temp_vector.at<double>(2, 0) = 1;
	temp_vector.at<double>(3, 0) = 1;
	for (int i = 0; i < 6; ++i)
	{
		transformations[i].copyTo(temp_transformation_matrix);
		temp_transformation_matrix.at<double>(0, 3) = 0;
		temp_transformation_matrix.at<double>(1, 3) = 0;
		temp_transformation_matrix.at<double>(2, 3) = 0;
		temp_transformation_matrix.at<double>(3, 3) = 1;
		cv::gemm(temp_transformation_matrix, temp_vector, 1, cv::Mat(), 0, normals[i]);
		//std::cout << "Normal " << i << " " << normals[i].at<double>(0, 0) << ", " << normals[i].at<double>(1, 0) << ", " << normals[i].at<double>(2, 0) << "\n";
	}
}

void CuboidTracker::calculateVisibleFaces()
{
	cv::Mat centerTranslation = centerTransform.col(3);
	for (int i = 0; i < 6; ++i)
	{
        faceIsVisible[i] = centerTranslation.dot(normals[i]) > 0.0;
	}
}

void CuboidTracker::setModel(int face, cv::Mat &face0, cv::Mat &face1, cv::Mat &face2, cv::Mat &face3, cv::Mat &face4, cv::Mat &face5)
{
    this->face = face;
    cv::Mat imageForModel = TrackingCommon::frame.clone();
    cv::Mat imageForModelOpposite = TrackingCommon::frame.clone();
    cv::Mat model;
    cv::Mat modelOpposite;
    cv::Rect modelSelection;
    // Set up texture for face 0 and 2
    modelSelection = cv::Rect(0, 0, face0.cols, face0.rows);
    model = imageForModel(modelSelection);
    modelOpposite = imageForModelOpposite(modelSelection);
    face0.copyTo(model);
    face2.copyTo(modelOpposite);
    this->trackers[0].setTargetObject(imageForModel, modelSelection);
    this->trackers[2].setTargetObject(imageForModelOpposite, modelSelection);

    // Set up texture for face 1 and 3
    modelSelection = cv::Rect(0, 0, face1.cols, face1.rows);
    model = imageForModel(modelSelection);
    modelOpposite = imageForModelOpposite(modelSelection);
    face1.copyTo(model);
    face3.copyTo(modelOpposite);
    this->trackers[1].setTargetObject(imageForModel, modelSelection);
    this->trackers[3].setTargetObject(imageForModelOpposite, modelSelection);

    // Set up texture for face 4 and 5
    modelSelection = cv::Rect(0, 0, face4.cols, face4.rows);
    model = imageForModel(modelSelection);
    modelOpposite = imageForModelOpposite(modelSelection);
    face4.copyTo(model);
    face5.copyTo(modelOpposite);
    this->trackers[4].setTargetObject(imageForModel, modelSelection);
    this->trackers[5].setTargetObject(imageForModelOpposite, modelSelection);
}

void CuboidTracker::reduceTransformationMatrix(cv::Mat &sourceMat, cv::Mat &destinationMat)
{
	destinationMat.at<double>(0, 0) = sourceMat.at<double>(0, 0);
	destinationMat.at<double>(1, 0) = sourceMat.at<double>(1, 0);
	destinationMat.at<double>(2, 0) = sourceMat.at<double>(2, 0);
	destinationMat.at<double>(0, 1) = sourceMat.at<double>(0, 1);
	destinationMat.at<double>(1, 1) = sourceMat.at<double>(1, 1);
	destinationMat.at<double>(2, 1) = sourceMat.at<double>(2, 1);
	destinationMat.at<double>(0, 2) = sourceMat.at<double>(0, 3);
	destinationMat.at<double>(1, 2) = sourceMat.at<double>(1, 3);
	destinationMat.at<double>(2, 2) = sourceMat.at<double>(2, 3);
}

void CuboidTracker::getHomographyParameters(int face, cv::Mat &parameters)
{
	parameters.at<double>(0, 0) = homographies[face].at<double>(0, 0);
	parameters.at<double>(1, 0) = homographies[face].at<double>(1, 0);
	parameters.at<double>(2, 0) = homographies[face].at<double>(2, 0);
	parameters.at<double>(3, 0) = homographies[face].at<double>(0, 1);
	parameters.at<double>(4, 0) = homographies[face].at<double>(1, 1);
	parameters.at<double>(5, 0) = homographies[face].at<double>(2, 1);
	parameters.at<double>(6, 0) = homographies[face].at<double>(0, 2);
	parameters.at<double>(7, 0) = homographies[face].at<double>(1, 2);
}

void CuboidTracker::initFacesMappings()
{
	facesMappings[0][0] = 0;
	facesMappings[0][1] = 1;
	facesMappings[0][2] = 0;
	facesMappings[0][3] = 1;
	facesMappings[0][4] = 2;
	facesMappings[0][5] = 2;

	facesMappings[1][0] = 1;
	facesMappings[1][1] = 0;
	facesMappings[1][2] = 1;
	facesMappings[1][3] = 0;
	facesMappings[1][4] = 2;
	facesMappings[1][5] = 2;

	facesMappings[4][0] = 2;
	facesMappings[4][1] = 1;
	facesMappings[4][2] = 2;
	facesMappings[4][3] = 1;
	facesMappings[4][4] = 0;
	facesMappings[4][5] = 0;

	for (int i = 0; i < 6; ++i)
	{
		facesMappings[2][i] = facesMappings[0][i];
		facesMappings[3][i] = facesMappings[1][i];
		facesMappings[5][i] = facesMappings[4][i];
	}
}

void CuboidTracker::setFacesDimensions(cv::Size2i face0, cv::Size2i face1, cv::Size2i face2)
{
    facesDimensions[0] = face0;
    facesDimensions[1] = face1;
    facesDimensions[2] = face0;
    facesDimensions[3] = face1;
    facesDimensions[4] = face2;
    facesDimensions[5] = face2;

    calculateBaseTransformations();
    calculateInverseBaseTransformations();
}

void CuboidTracker::setRealFacesDimensions(cv::Size2f face0, cv::Size2f face1, cv::Size2f face2)
{
    realFacesDimensions[0] = face0;
    realFacesDimensions[1] = face1;
    realFacesDimensions[2] = face0;
    realFacesDimensions[3] = face1;
    realFacesDimensions[4] = face2;
    realFacesDimensions[5] = face2;

    // Estimate scalings between realw world and tracked faces
    for (int i = 0; i < 6; ++i) {
        //realWorldToTemplateScalings[i] = cv::Size2f(realFacesDimensions[i].width / (1.0 * facesDimensions[i].width), realFacesDimensions[i].height / (1.0 * facesDimensions[i].height));
        //realWorldToTemplateScalings[i] = cv::Size2f(realFacesDimensions[i].width / (1.0 * facesDimensions[i].width), realFacesDimensions[i].height / (1.0 * facesDimensions[i].height));
        realWorldToTemplateScalings[i] = cv::Size2f(realFacesDimensions[i].width / (1.0 * facesDimensions[i].width), realFacesDimensions[i].height / (1.0 * facesDimensions[i].height));
    }

    useRealWorldDimensions = true;
    calculateBaseTransformations();
    calculateInverseBaseTransformations();
}

int CuboidTracker::mapFaceFromBase(int base, int face)
{
	return facesMappings[base][face];
}

ESMTracker* CuboidTracker::getTracker(int index)
{
	return &trackers[index];
}

void CuboidTracker::calculateBaseTransformations()
{
	cv::Mat applyTransform = cv::Mat(4, 4, CV_64F);
	cv::setIdentity(applyTransform);
	cv::Mat applyTranslation = cv::Mat(4, 4, CV_64F);
	cv::setIdentity(applyTranslation);

	// Cuboid center
	cv::setIdentity(baseToCenterTransform);

    cv::Size2f dimensions[6];
    for (int i = 0; i < 6; ++i)
    {
        dimensions[i] = facesDimensions[i];
    }

    baseToCenterTransform.at<double>(0, 3) = (double)dimensions[0].width / 2;
    baseToCenterTransform.at<double>(1, 3) = (double)dimensions[0].height / 2;
    baseToCenterTransform.at<double>(2, 3) = (double)dimensions[4].width / 2;
    cv::invert(baseToCenterTransform, centerToBaseTransform);

	// Face 1 (Top)
    cv::gemm(applyTransform, rotateMinusX, 1, cv::Mat(), 0, baseTransformations[1]);
    cv::gemm(baseTransformations[1], rotateMinusZ, 1, cv::Mat(), 0, baseTransformations[1]);

	// Face 2 (Behind)
	baseTransformations[0].copyTo(baseTransformations[2]);
	cv::gemm(baseTransformations[2], rotateMinusX, 1, cv::Mat(), 0, baseTransformations[2]);
	cv::gemm(baseTransformations[2], rotateMinusX, 1, cv::Mat(), 0, baseTransformations[2]);
    applyTranslation.at<double>(0, 3) = (double)dimensions[0].width;
	applyTranslation.at<double>(1, 3) = 0.0;
    applyTranslation.at<double>(2, 3) = (double)-dimensions[1].width;
	cv::gemm(baseTransformations[2], applyTranslation, 1, cv::Mat(), 0, baseTransformations[2]);
	cv::gemm(baseTransformations[2], rotateMinusZ, 1, cv::Mat(), 0, baseTransformations[2]);
	cv::gemm(baseTransformations[2], rotateMinusZ, 1, cv::Mat(), 0, baseTransformations[2]);

	// Face 3 (Bottom)
	cv::gemm(applyTransform, rotateX, 1, cv::Mat(), 0, baseTransformations[3]);
	applyTranslation.at<double>(0, 3) = 0.0;
    applyTranslation.at<double>(1, 3) = (double)dimensions[3].width;
    applyTranslation.at<double>(2, 3) = (double)-dimensions[0].height;
	cv::gemm(baseTransformations[3], applyTranslation, 1, cv::Mat(), 0, baseTransformations[3]);
	cv::gemm(baseTransformations[3], rotateMinusZ, 1, cv::Mat(), 0, baseTransformations[3]);
	//cv::gemm(baseTransformations[3], rotateMinusX, 1, cv::Mat(), 0, baseTransformations[3]);
	//cv::gemm(baseTransformations[3], rotateMinusX, 1, cv::Mat(), 0, baseTransformations[3]);

	// Face 4 (Left)
	cv::gemm(baseTransformations[0], rotateY, 1, cv::Mat(), 0, baseTransformations[4]);
    applyTranslation.at<double>(0, 3) = (double)-dimensions[4].width;
	applyTranslation.at<double>(1, 3) = 0.0;
	applyTranslation.at<double>(2, 3) = 0.0;
	cv::gemm(baseTransformations[4], applyTranslation, 1, cv::Mat(), 0, baseTransformations[4]);

	// Face 5 (Right)
	cv::gemm(baseTransformations[0], rotateMinusY, 1, cv::Mat(), 0, baseTransformations[5]);
	applyTranslation.at<double>(0, 3) = 0.0;
	applyTranslation.at<double>(1, 3) = 0.0;
    applyTranslation.at<double>(2, 3) = (double)-dimensions[0].width;
	cv::gemm(baseTransformations[5], applyTranslation, 1, cv::Mat(), 0, baseTransformations[5]);
}

void CuboidTracker::calculateInverseBaseTransformations()
{
	for (int i = 1; i < 6; ++i)
		cv::invert(baseTransformations[i], baseInverseTransformations[i]);
}

