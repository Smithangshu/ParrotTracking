#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QSettings>
#include <QDebug>
#include <QImage>
#include <QProcess>
#include <Phonon/MediaObject>
//#include <octave/oct.h>
#include <Phonon/AudioOutput>
#include "common/TrackingCommon.h"
#include "common/calibration.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    populateTestsCombobox();
    QSettings settings("3DTracker", "ITESM/ERobots");

    ui->mainToolBox->setCurrentIndex(settings.value("GUI/ToolboxPosition", 0).toInt());

    ui->cameraLineEdit->setText(settings.value("GUI/CameraIndex", "0").toString());

    TrackingCommon::videoFileName = settings.value("GUI/VideoFileName", "").toString().toStdString();
    ui->selectMediaLabel->setText(settings.value("GUI/VideoFileName", "").toString());

    ui->serialPortLineEdit->setText(settings.value("GUI/SerialPort", "/dev/ttyUSB0").toString());

    TrackingCommon::pauseOnFirstFrame = settings.value("GUI/PauseOnFirstFrame", false).toBool();
    ui->pauseOnFirstFrameCheckBox->setChecked(TrackingCommon::pauseOnFirstFrame);
    TrackingCommon::pauseOnEachFrame = settings.value("GUI/PauseOnEachFrame", false).toBool();
    ui->pauseOnEachFrameCheckBox->setChecked(TrackingCommon::pauseOnEachFrame);
    TrackingCommon::enableDeinterlace = settings.value("GUI/Deinterlace", false).toBool();
    ui->interlaceCheckBox->setChecked(TrackingCommon::enableDeinterlace);

    TrackingCommon::homePan = settings.value("GUI/CAM/Pan", "0").toInt();
    ui->panSlider->setValue(TrackingCommon::homePan);

    TrackingCommon::homeTilt = settings.value("GUI/CAM/Tilt", "0").toInt();
    ui->tiltSlider->setValue(TrackingCommon::homeTilt);

    TrackingCommon::homeZoom = settings.value("GUI/CAM/Zoom", "0").toInt();
    ui->zoomSlider->setValue(TrackingCommon::homeZoom);

    TrackingCommon::homeSpeed = settings.value("GUI/CAM/Speed", "2").toInt();
    ui->speedDial->setValue(TrackingCommon::homeSpeed);

    TrackingCommon::homeFocus = settings.value("GUI/CAM/Focus", 0).toInt();
    ui->focusSlider->setValue(TrackingCommon::homeFocus);

    TrackingCommon::deltaPanTilt = settings.value("GUI/CAM/DeltaPanTilt", 100).toInt();
    ui->deltaPanTiltSlider->setValue(TrackingCommon::deltaPanTilt);

    TrackingCommon::deltaXYThreshold = settings.value("GUI/CAM/DeltaXYThreshold", 10).toInt();
    ui->cameraCenterThreshold->setValue(TrackingCommon::deltaXYThreshold);

    // Program Mode
    ui->meanShiftRadio->setChecked(settings.value("GUI/ProgramMeanShift", false).toBool());
    ui->ESMRadio->setChecked(settings.value("GUI/ProgramESM", false).toBool());
    ui->cuboidTrackingRadio->setChecked(settings.value("GUI/ProgramCuboid", false).toBool());

    if (settings.value("GUI/SerialPortStatus", 0).toBool())
    {
        qDebug() << "Reconnecting Serial";
        connectSerialPort();
    }
    TrackingCommon::lastUsedCalibration = settings.value("GUI/LastUsedCalibration", "").toString().toStdString();
    if (TrackingCommon::lastUsedCalibration != "")
    {
        if (mainController.loadCalibration(TrackingCommon::lastUsedCalibration))
        {
            ui->loadedCalibrationLabel->setText(tr("[%1]").arg(QString(TrackingCommon::lastUsedCalibration.c_str())));
            ui->viewCalibrationButton->setEnabled(true);
            ui->clearCalibrationButton->setEnabled(true);
            TrackingCommon::mainController->updateCameraCalibrationInterpolation(ui->zoomSlider->value());
        }
    }
    TrackingCommon::lastUsedHomography = settings.value("GUI/LastUsedHomography", "").toString().toStdString();
    if (TrackingCommon::lastUsedHomography != "")
    {
        if (mainController.loadHomography(TrackingCommon::lastUsedHomography))
        {
            ui->loadedHomographyLabel->setText(tr("[%1]").arg(QString(TrackingCommon::lastUsedHomography.c_str())));
            ui->viewHomographyButton->setEnabled(true);
            ui->clearHomographyButton->setEnabled(true);
        }
    }
    TrackingCommon::glDelay = settings.value("GUI/GLDelay", 105).toInt();
    ui->glDelaySlider->setValue(TrackingCommon::glDelay);

    ui->cameraRadio->setChecked(settings.value("GUI/CameraRadio", false).toBool());
    ui->mediaRadio->setChecked(settings.value("GUI/MediaRadio", false).toBool());
    if (ui->cameraRadio->isChecked()) {
        mainController.setVideoMode(MainController::VIDEO_CAMERA);
    } else {
        mainController.setVideoMode(MainController::VIDEO_FILE);        void setTrajectorySignal(cv::Mat trajectory);

    }

    // 2D Localization
    TrackingCommon::enableHeightCorrection = settings.value("GUI/EnableHeightCorrection", false).toBool();
    ui->heightCheckBox->setChecked(TrackingCommon::enableHeightCorrection);
    TrackingCommon::enableScalingWithHomography = settings.value("GUI/EnableScalingWithHomography", false).toBool();
    ui->scalingHomographyCheckBox->setChecked(TrackingCommon::enableScalingWithHomography);
    TrackingCommon::enableHomographyInterpolation = settings.value("GUI/EnableHomographyInterpolation", false).toBool();
    ui->interpolationCheckBox->setChecked(TrackingCommon::enableHomographyInterpolation);
    TrackingCommon::objectHeight = settings.value("GUI/ObjectHeight", "0.0").toFloat();
    ui->heightLineEdit->setText(settings.value("GUI/ObjectHeight", "0.0").toString());
    TrackingCommon::recordPositionEstimation = settings.value("GUI/RecordPositionEstimation", false).toBool();
    ui->recordPositionEstimation->setChecked(TrackingCommon::recordPositionEstimation);
    TrackingCommon::displayUnscaled = settings.value("GUI/DisplayUnscaled", false).toBool();
    ui->displayUnscaledCheckBox->setChecked(TrackingCommon::displayUnscaled);
    TrackingCommon::enableSendingPositionData = settings.value("GUI/EnableSendingPositionData", false).toBool();
    ui->sendPositionDataCheckBox->setChecked(TrackingCommon::enableSendingPositionData);

    // Plane Detection
    TrackingCommon::enableMotionFilter = settings.value("GUI/EnableMotionFilter", false).toBool();
    ui->enableMotionFilterCheckBox->setChecked(TrackingCommon::enableMotionFilter);
    TrackingCommon::searchPlaneLines = settings.value("GUI/SearchPlaneLines", false).toBool();
    ui->linePlaneSearchCheckBox->setChecked(TrackingCommon::searchPlaneLines);
    TrackingCommon::searchPlaneLinesRecord = settings.value("GUI/SearchPlaneLinesRecord", false).toBool();
    ui->showLinesPoseCheckBox->setChecked(TrackingCommon::searchPlaneLinesRecord);

    TrackingCommon::areaThreshold = settings.value("GUI/AreaThreshold", "100").toInt();
    ui->areaThresholdSlider->setValue(TrackingCommon::areaThreshold);
    TrackingCommon::areaMaxThreshold = settings.value("GUI/AreaMaxThreshold", "100").toInt();
    ui->areaMaxThresholdSlider->setValue(TrackingCommon::areaMaxThreshold);
    TrackingCommon::colorBrightnessThreshold = settings.value("GUI/ColorBrightnessThreshold", "200").toInt();
    ui->colorBrightnessThresholdSlider->setValue(TrackingCommon::colorBrightnessThreshold);
    TrackingCommon::colorHSVThreshold = settings.value("GUI/ColorHSVThreshold", "220").toInt();
    ui->colorHSVThresholdSlider->setValue(TrackingCommon::colorHSVThreshold);

    // Other
    TrackingCommon::enableFilteringMeanShift = settings.value("GUI/EnableFilteringMeanShift", false).toBool();
    ui->filterMeanShift->setChecked(TrackingCommon::enableFilteringMeanShift);
    TrackingCommon::enableFiltering3DPose = settings.value("GUI/EnableFiltering3DPose", false).toBool();
    ui->enable3DTrackerFiltering->setChecked(TrackingCommon::enableFiltering3DPose);
    ui->testsComboBox->setCurrentIndex(settings.value("GUI/SelectedTest", 0).toInt());
    ui->cuboidSelectedFaceComboBox->setCurrentIndex(settings.value("GUI/Cuboid/SelectedFace", 0).toInt());
    ui->cuboidModelsComboBox->setCurrentIndex(settings.value("GUI/Cuboid/SelectedModel", 0).toInt());
    TrackingCommon::enableLockFace = settings.value("GUI/Cuboid/EnableLockFace", false).toBool();
    ui->lockFaceCheckBox->setChecked(TrackingCommon::enableLockFace);
    TrackingCommon::enableMultipleFaceTracking = settings.value("GUI/Cuboid/EnableMultipleFaceTracking", false).toBool();
    ui->multiFaceTrackingCheckBox->setChecked(TrackingCommon::enableMultipleFaceTracking);
    TrackingCommon::maxTrackedFaces = settings.value("GUI/Cuboid/MaxTrackedFaces", 1).toInt();
    ui->maxTrackedFaces->setText(QString::number(TrackingCommon::maxTrackedFaces));

    TrackingCommon::selfRecover = settings.value("GUI/SelfRecover", false).toBool();
    ui->selfRecoverCheckBox->setChecked(TrackingCommon::selfRecover);

    // Other
    ui->linesWidthLineEdit->setText(settings.value("GUI/LinesWidth", "0.0").toString());
    ui->linesHeightLineEdit->setText(settings.value("GUI/LinesHeight", "0.0").toString());
    ui->lineWidthLineEdit->setText(settings.value("GUI/LineWidth", "0.0").toString());

    // GP Serial port
    ui->gpSerialPortLineEdit->setText(settings.value("GUI/GPSerialPortName", "/dev/ttyUSB1").toString());
    if (settings.value("GUI/GPSerialPortStatus", false).toBool())
    {
        connectGPSerialPort();
    }

    // Populate list of joysticks
    for (int i = 0; i < TrackingCommon::joypadController.availableJoysticks(); ++i)
    {
        ui->joypadsComboBox->addItem(TrackingCommon::joypadController.joystickName(i));
    }

    // Add header for parrot points table
    QStringList tableHeader = QStringList();
    tableHeader.push_back("S");
    tableHeader.push_back("X");
    tableHeader.push_back("Y");
    tableHeader.push_back("Z");
    tableHeader.push_back("Yaw");
    ui->parrotPointsTable->setColumnCount(5);
    ui->parrotPointsTable->setHorizontalHeaderLabels(tableHeader);
    ui->parrotPointsTable->setColumnWidth(0, 30);
    ui->parrotPointsTable->setColumnWidth(1, 58);
    ui->parrotPointsTable->setColumnWidth(2, 58);
    ui->parrotPointsTable->setColumnWidth(3, 58);
    ui->parrotPointsTable->setColumnWidth(4, 58);

    ui->parrotControlVariablesFilename->setText(settings.value("GUI/ControlVarsFile").toString());
    loadParrotControlVariables(settings.value("GUI/ControlVarsFile").toString().toStdString());

    TrackingCommon::homographyCaptureWindow = new HomographyCaptureWindow();
    TrackingCommon::dataDisplayerWindow = new DataDisplayerWindow();
    TrackingCommon::mainController = &this->mainController;
    cv::setIdentity(TrackingCommon::objectZero);

    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<std::string>("std::string");
    connect(&mainController, SIGNAL(imageWindowReadySignal(std::string, const QImage &)), this, SLOT(updateImageWindow(std::string, const QImage &)));
    connect(&mainController, SIGNAL(imageWindowsDestroySignal()), this, SLOT(destroyImageWindows()));
    qRegisterMetaType<WindowCallBack*>("WindowCallBack*");
    connect(&mainController, SIGNAL(imageWindowMouseCallBackSignal(std::string, WindowCallBack*)), this, SLOT(setImageWindowsMouseCallBack(std::string, WindowCallBack*)));
    qRegisterMetaType<DataDisplayer::DataDisplayModes>("DataDisplayer::DataDisplayModes");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(&mainController, SIGNAL(dataDisplayerSignal(std::string, cv::Mat, DataDisplayer::DataDisplayModes)), this, SLOT(displayData(std::string, cv::Mat, DataDisplayer::DataDisplayModes)));
    connect(&mainController, SIGNAL(displayCoordinatesSignal(std::string, std::string, cv::Mat)), this, SLOT(displayCoordinates(std::string, std::string, cv::Mat)));
    connect(&mainController, SIGNAL(displayPoseSignal(std::string, std::string, cv::Mat)), this, SLOT(displayPose(std::string, std::string, cv::Mat)));

    connect(&mainController, SIGNAL(enable3DReconstructionSignal(bool)), this, SLOT(enable3DReconstruction(bool)));
    connect(&mainController, SIGNAL(enableSaveHomographySignal(bool)), this, SLOT(enableSaveHomography(bool)));
    connect(&mainController, SIGNAL(enableMultiFaceTracking(bool)), this, SLOT(enableMultiFaceTracking(bool)));

    connect(&mainController, SIGNAL(displayParrotControlValuesSignal(int,int,int,int)), this, SLOT(displayParrotControlValues(int,int,int,int)));
    connect(&TrackingCommon::mainSocket, SIGNAL(displayHapticInputSignal(int,int)), this, SLOT(displayHapticInputValues(int,int)));
    connect(&TrackingCommon::parrotController, SIGNAL(setBatteryChargeValueSignal(int)), this, SLOT(displayBatteryCharge(int)));
    connect(&mainController, SIGNAL(connectParrotSignal()), &TrackingCommon::parrotController, SLOT(connectSlot()));
    connect(&mainController, SIGNAL(disconnectParrotSignal()), &TrackingCommon::parrotController, SLOT(disconnectSlot()));
    connect(&TrackingCommon::joypadController, SIGNAL(setJoypadValuesSignal(float,float,float,float)), &TrackingCommon::parrotController, SLOT(setJoypadValuesSlot(float,float,float,float)));
    connect(&mainController, SIGNAL(takeOffParrotSignal()), &TrackingCommon::parrotController, SLOT(takeOffSlot()));
    connect(&mainController, SIGNAL(landParrotSignal()), &TrackingCommon::parrotController, SLOT(landSlot()));
    connect(&mainController, SIGNAL(setParrotValuesSignal(float,float,float,float,float,float)), &TrackingCommon::parrotController, SLOT(setAnglesValuesSlot(float,float,float,float,float,float)));
    connect(&mainController, SIGNAL(setParrotPoseSignal(cv::Mat)), &TrackingCommon::parrotController, SLOT(setPoseSlot(cv::Mat)));
    connect(&mainController, SIGNAL(setZeroSignal()), &mainController.glThread, SLOT(setZeroSlot()));
    connect(&mainController, SIGNAL(setTrajectorySignal(cv::Mat)), &TrackingCommon::parrotController, SLOT(setTrajectorySlot(cv::Mat)));
    connect(&mainController, SIGNAL(resetParrotFilterSignal(cv::Mat)), &TrackingCommon::parrotController, SLOT(resetFilterSlot(cv::Mat)));
    connect(&mainController, SIGNAL(setParrotControlVariablesSignal(float,float,float,float,float,float,float,float,float,float,float,float,float,float,float)), &TrackingCommon::parrotController, SLOT(setControlVariablesSlot(float,float,float,float,float,float,float,float,float,float,float,float,float,float,float)));
    connect(&mainController, SIGNAL(getParrotControlVariablesSignal(float&,float&,float&,float&,float&,float&,float&,float&,float&,float&,float&,float&)), &TrackingCommon::parrotController, SLOT(getControlVariablesSlot(float&,float&,float&,float&,float&,float&,float&,float&,float&,float&,float&,float&)));
    connect(&mainController, SIGNAL(updateParrotTrajectoryStatusSignal(int,bool)), this, SLOT(updateParrotTrajectoryStatus(int,bool)));
    connect(&mainController, SIGNAL(displayCVImageSignal(std::string,cv::Mat)), this, SLOT(displayCVImage(std::string,cv::Mat)));

    connect(&mainController, SIGNAL(setParrotAutomaticControlSignal(bool)), this, SLOT(setParrotAutomaticControl(bool)));
    connect(&mainController, SIGNAL(setParrotConnectionStatusSignal(bool)), this, SLOT(setParrotConnectionStatus(bool)));
    connect(&mainController, SIGNAL(create3DReconstructionWindowSignal()), this, SLOT(create3DReconstructionWindow()));

}

MainWindow::~MainWindow()
{
    QSettings settings("3DTracker", "ITESM/ERobots");
    settings.setValue("GUI/ToolboxPosition", ui->mainToolBox->currentIndex());
    settings.setValue("GUI/CameraIndex", ui->cameraLineEdit->text());
    settings.setValue("GUI/VideoFileName", TrackingCommon::videoFileName.c_str());
    settings.setValue("GUI/SerialPort", ui->serialPortLineEdit->text());
    settings.setValue("GUI/SerialPortStatus", TrackingCommon::pTZCameraController.isConnected());
    settings.setValue("GUI/PauseOnFirstFrame", TrackingCommon::pauseOnFirstFrame);
    settings.setValue("GUI/PauseOnEachFrame", TrackingCommon::pauseOnEachFrame);
    settings.setValue("GUI/Deinterlace", TrackingCommon::enableDeinterlace);
    settings.setValue("GUI/CAM/Pan", TrackingCommon::homePan);
    settings.setValue("GUI/CAM/Tilt", TrackingCommon::homeTilt);
    settings.setValue("GUI/CAM/Zoom", TrackingCommon::homeZoom);
    settings.setValue("GUI/CAM/Focus", TrackingCommon::homeFocus);
    settings.setValue("GUI/CAM/FocusAutoOn", TrackingCommon::focusAutoOn);
    settings.setValue("GUI/CAM/Speed", TrackingCommon::homeSpeed);
    settings.setValue("GUI/CAM/DeltaPanTilt", TrackingCommon::deltaPanTilt);
    settings.setValue("GUI/CAM/DeltaXYThreshold", TrackingCommon::deltaXYThreshold);
    settings.setValue("GUI/LastUsedHomography", QString(TrackingCommon::lastUsedHomography.c_str()));
    settings.setValue("GUI/LastUsedCalibration", QString(TrackingCommon::lastUsedCalibration.c_str()));
    settings.setValue("GUI/GLDelay", TrackingCommon::glDelay);
    // Program mode
    settings.setValue("GUI/ProgramMeanShift", ui->meanShiftRadio->isChecked());
    settings.setValue("GUI/ProgramESM", ui->ESMRadio->isChecked());
    settings.setValue("GUI/ProgramCuboid", ui->cuboidTrackingRadio->isChecked());

    // Media type selection
    settings.setValue("GUI/CameraRadio", ui->cameraRadio->isChecked());
    settings.setValue("GUI/MediaRadio", ui->mediaRadio->isChecked());
    // 2D Localization
    settings.setValue("GUI/EnableHeightCorrection", TrackingCommon::enableHeightCorrection);
    settings.setValue("GUI/EnableScalingWithHomography", TrackingCommon::enableScalingWithHomography);
    settings.setValue("GUI/EnableHomographyInterpolation", TrackingCommon::enableHomographyInterpolation);
    settings.setValue("GUI/ObjectHeight", TrackingCommon::objectHeight);
    settings.setValue("GUI/RecordPositionEstimation", TrackingCommon::recordPositionEstimation);
    settings.setValue("GUI/DisplayUnscaled", TrackingCommon::displayUnscaled);
    settings.setValue("GUI/EnableSendingPositionData", TrackingCommon::enableSendingPositionData);
    // Plane Detection
    settings.setValue("GUI/EnableMotionFilter", TrackingCommon::enableMotionFilter);
    settings.setValue("GUI/SearchPlaneLines", TrackingCommon::searchPlaneLines);
    settings.setValue("GUI/SearchPlaneLinesRecord", TrackingCommon::searchPlaneLinesRecord);
    settings.setValue("GUI/AreaThreshold", TrackingCommon::areaThreshold);
    settings.setValue("GUI/AreaMaxThreshold", TrackingCommon::areaMaxThreshold);
    settings.setValue("GUI/ColorBrightnessThreshold", TrackingCommon::colorBrightnessThreshold);
    settings.setValue("GUI/ColorHSVThreshold", TrackingCommon::colorHSVThreshold);
    // Other
    settings.setValue("GUI/EnableFilteringMeanShift", TrackingCommon::enableFilteringMeanShift);
    settings.setValue("GUI/EnableFiltering3DPose", TrackingCommon::enableFiltering3DPose);
    settings.setValue("GUI/SelectedTest", ui->testsComboBox->currentIndex());
    settings.setValue("GUI/SelfRecover", TrackingCommon::selfRecover);
    settings.setValue("GUI/Cuboid/SelectedFace", TrackingCommon::cuboidSelectedFace);
    settings.setValue("GUI/Cuboid/SelectedModel", TrackingCommon::cuboidSelectedModel);
    settings.setValue("GUI/Cuboid/EnableLockFace", TrackingCommon::enableLockFace);
    settings.setValue("GUI/Cuboid/EnableMultipleFaceTracking", TrackingCommon::enableMultipleFaceTracking);
    settings.setValue("GUI/Cuboid/MaxTrackedFaces", TrackingCommon::maxTrackedFaces);

    // GP Serial Port
    settings.setValue("GUI/GPSerialPortName", ui->gpSerialPortLineEdit->text());
    settings.setValue("GUI/GPSerialPortStatus", TrackingCommon::gpSerialPort->isOpen());
    // Parrot
    settings.setValue("GUI/ControlVarsFile", ui->parrotControlVariablesFilename->text());

    // Settings
    settings.setValue("GUI/LinesWidth", ui->linesWidthLineEdit->text());
    settings.setValue("GUI/LinesHeight", ui->linesHeightLineEdit->text());
    settings.setValue("GUI/LineWidth", ui->lineWidthLineEdit->text());

    disconnectGPSerialPort();
    mainController.disconnectJoypad(0);
    mainController.glThread.terminate();
    TrackingCommon::pTZCameraController.disconnect();
    //TrackingCommon::parrotController.disconnect();
    if (TrackingCommon::parrotController.isRunning())
        TrackingCommon::parrotController.terminate();
    if (TrackingCommon::pTZCameraController.isRunning())
        TrackingCommon::pTZCameraController.terminate();
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *)
{
    qDebug() << "Closing";
    mainController.quit();

    if (TrackingCommon::dataDisplayerWindow->isVisible())
        TrackingCommon::dataDisplayerWindow->close();

    if (TrackingCommon::homographyCaptureWindow->isVisible())
        TrackingCommon::homographyCaptureWindow->close();

    // Also Shutdown remaining OpenCV windows
    cv::destroyAllWindows();

    mainController.destroyImageWindows();
}

void MainWindow::populateTestsCombobox()
{
    for (int i = 0; i < TrackingCommon::testsController.totalTests; ++i)
    {
        ui->testsComboBox->insertItem(i, tr(TrackingCommon::testsController.tests[i].testString.c_str()));
    }
    for (int i = 0; i < TrackingCommon::testsController.totalModels; ++i)
    {
        ui->cuboidModelsComboBox->insertItem(i, tr(TrackingCommon::testsController.models[i].testString.c_str()));
    }
}


void MainWindow::updateImageWindow(string windowTitle, const QImage &image)
{
    mainController.prepareImageWindow(windowTitle);
    mainController.imageWindows[windowTitle]->displayImage(image);
}

void MainWindow::destroyImageWindows()
{
    mainController.destroyImageWindows();
}

void MainWindow::setImageWindowsMouseCallBack(std::string windowTitle, WindowCallBack* windowCallBack)
{
    mainController.setImageWindowMouseCallBack(windowTitle, windowCallBack);
}

void MainWindow::displayData(std::string tabName, cv::Mat mat, DataDisplayer::DataDisplayModes dataDisplayMode)
{
    TrackingCommon::dataDisplayer.displayData(tabName, mat, dataDisplayMode);
}

void MainWindow::displayCoordinates(std::string tabName, std::string displayName, cv::Mat displayData)
{
    TrackingCommon::dataDisplayer.displayCoordinates(tabName, displayName, displayData);
}

void MainWindow::displayPose(std::string tabName, std::string displayName, cv::Mat displayData)
{
    TrackingCommon::dataDisplayer.displayPose(tabName, displayName, displayData);
}

void MainWindow::on_startVideoButton_clicked()
{
    ui->stopVideoButton->setEnabled(true);
    ui->pauseVideoButton->setEnabled(true);
    if (mainController.programStatus != MainController::VIDEO_PAUSED)
    {
        mainController.startVideo();
        mainController.start();
    }
    else
    {
        mainController.programStatus = MainController::VIDEO_PLAYING;
    }
}

void MainWindow::on_stopVideoButton_clicked()
{
    ui->stopVideoButton->setEnabled(false);
    ui->pauseVideoButton->setEnabled(false);
    mainController.stopVideo();
}

void MainWindow::on_cameraLineEdit_textChanged(const QString &arg1)
{
    mainController.setCameraIndex(arg1.toInt());
}

void MainWindow::on_selectMediaButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    QString(),
                                                    tr("Videos (*.mp4 *.avi *.mpeg *.ogv *.mkv)"));
    if (fileName != "")
    {
        mainController.setMediaFileName(fileName.toStdString());
        ui->selectMediaLabel->setText(fileName);
    }
}

void MainWindow::on_cameraRadio_clicked(bool checked)
{
    if (checked)
    {
        mainController.setVideoMode(MainController::VIDEO_CAMERA);
    }
}

void MainWindow::on_mediaRadio_clicked(bool checked)
{
    if (checked)
    {
        mainController.setVideoMode(MainController::VIDEO_FILE);
    }
}

void MainWindow::on_launchTestButton_clicked()
{
    int testIndex = ui->testsComboBox->currentIndex();
    mainController.testMode = true;
    mainController.testIndex = testIndex;

    if (TrackingCommon::testsController.tests[testIndex].videoMode == TestsController::CAPTURE_VIDEO)
    {
        std::string mediaFileName = TrackingCommon::testsController.basePath;
        mediaFileName += TrackingCommon::testsController.tests[testIndex].videoFileName;
        ui->selectMediaLabel->setText(tr(mediaFileName.c_str()));
        mainController.setMediaFileName(mediaFileName);
        mainController.setVideoMode(MainController::VIDEO_FILE);

        ui->stopVideoButton->setEnabled(true);
        ui->pauseVideoButton->setEnabled(true);
        mainController.startVideo();
        mainController.start();
    } else if (TrackingCommon::testsController.tests[testIndex].videoMode == TestsController::CAPTURE_IMAGE) {
        TrackingCommon::testsController.launchTest(testIndex);
    }
}

void MainWindow::on_pauseOnFirstFrameCheckBox_clicked(bool checked)
{
    TrackingCommon::pauseOnFirstFrame = checked;
}

void MainWindow::on_pauseVideoButton_clicked()
{
    mainController.pauseVideo();
}

void MainWindow::on_ESMRadio_clicked(bool checked)
{
    if (checked)
    {
        TrackingCommon::trackingMode = TrackingCommon::TRACKING_ESM;
        TrackingCommon::programMode = TrackingCommon::TRACKING;
    }
}

void MainWindow::on_meanShiftRadio_clicked(bool checked)
{
    if (checked)
    {
        TrackingCommon::trackingMode = TrackingCommon::TRACKING_MS;
        TrackingCommon::programMode = TrackingCommon::TRACKING;
    }
}

void MainWindow::on_LKAdditiveRadio_clicked(bool checked)
{

}

void MainWindow::on_LKInverseRadio_clicked(bool checked)
{

}

void MainWindow::on_zoomSlider_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setZoom(value);
    TrackingCommon::mainController->updateCameraCalibrationInterpolation(value);
}

void MainWindow::connectSerialPort()
{
    QIcon icon;
    TrackingCommon::pTZCameraController.setSerialPort(ui->serialPortLineEdit->text().toStdString());
    TrackingCommon::pTZCameraController.connect();
    if (TrackingCommon::pTZCameraController.isConnected())
    {
        icon.addFile(QString::fromUtf8("../disconnect.png"), QSize(), QIcon::Normal, QIcon::Off);
        ui->serialPortConnectButton->setIcon(icon);
        ui->panSlider->setEnabled(true);
        ui->tiltSlider->setEnabled(true);
        ui->zoomSlider->setEnabled(true);
        TrackingCommon::pTZCameraController.setPanTiltSpeed(ui->speedDial->value(), ui->speedDial->value());
        TrackingCommon::positionEstimatorController.setCameraSpeed(ui->speedDial->value(), ui->speedDial->value());
        TrackingCommon::pTZCameraController.setPanTilt(ui->panSlider->value(), ui->tiltSlider->value());
        TrackingCommon::pTZCameraController.setZoom(ui->zoomSlider->value());
        ui->centerCameraCheckBox->setEnabled(true);
        // Start thread for reading packages
        TrackingCommon::pTZCameraController.start();
    }
    else
    {
        qDebug() << "Can't connect to serial port";
    }
}

void MainWindow::disconnectSerialPort()
{
    QIcon icon;
    ui->serialPortConnectButton->setIcon(icon);
    TrackingCommon::pTZCameraController.disconnect();
    if (!TrackingCommon::pTZCameraController.isConnected())
    {
        icon.addFile(QString::fromUtf8("../connect.png"), QSize(), QIcon::Normal, QIcon::Off);
        ui->serialPortConnectButton->setIcon(icon);
        ui->panSlider->setEnabled(false);
        ui->tiltSlider->setEnabled(false);
        ui->zoomSlider->setEnabled(false);
        ui->centerCameraCheckBox->setEnabled(false);
    }
}

void MainWindow::connectGPSerialPort()
{
    QIcon icon;
    if (!TrackingCommon::gpSerialPort->isOpen())
    {
        if (mainController.connectGPSerialPort())
        {
            icon.addFile(QString::fromUtf8("../disconnect.png"), QSize(), QIcon::Normal, QIcon::Off);
            ui->gppSerialPortButton->setIcon(icon);
        }
    }
    else
    {
        qDebug() << "GP Serial Port is Open";
    }
}

void MainWindow::disconnectGPSerialPort()
{
    QIcon icon;
    icon.addFile(QString::fromUtf8("../connect.png"), QSize(), QIcon::Normal, QIcon::Off);
    ui->gppSerialPortButton->setIcon(icon);
    mainController.disconnectGPSerialPort();
}

void MainWindow::showEvent(QShowEvent *)
{
    mainController.connectJoypad(0);
    ui->applyControlBarsButton->click();
}


void MainWindow::on_serialPortConnectButton_clicked()
{
    if (!TrackingCommon::pTZCameraController.isConnected())
    {
        connectSerialPort();
    }
    else
    {
        disconnectSerialPort();
    }
}

void MainWindow::on_panSlider_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setPan(value);
    if (TrackingCommon::positionEstimatorController.isReady() && TrackingCommon::calibrated == true) {
        TrackingCommon::positionEstimatorController.updateTargetHomography(TrackingCommon::pTZCameraController.pan, TrackingCommon::pTZCameraController.tilt, true, true);
    }
}

void MainWindow::on_tiltSlider_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setTilt(value);
    if (TrackingCommon::positionEstimatorController.isReady() && TrackingCommon::calibrated == true) {
        TrackingCommon::positionEstimatorController.updateTargetHomography(TrackingCommon::pTZCameraController.pan, TrackingCommon::pTZCameraController.tilt, true, true);
    }
}

void MainWindow::on_speedDial_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setPanTiltSpeed(value, value);
    TrackingCommon::positionEstimatorController.setCameraSpeed(value, value);
}

void MainWindow::on_cameraSetHomeButton_clicked()
{
    TrackingCommon::homePan = ui->panSlider->value();
    TrackingCommon::homeTilt = ui->tiltSlider->value();
    TrackingCommon::homeSpeed = ui->speedDial->value();
    TrackingCommon::homeZoom = ui->zoomSlider->value();
    TrackingCommon::deltaPanTilt = ui->deltaPanTiltSlider->value();
    TrackingCommon::deltaXYThreshold = ui->cameraCenterThreshold->value();
}

void MainWindow::on_cameraResetButton_clicked()
{
    ui->panSlider->setValue(0);
    ui->tiltSlider->setValue(0);
    ui->speedDial->setValue(2);
    ui->zoomSlider->setValue(0);
    ui->deltaPanTiltSlider->setValue(100);
    ui->cameraCenterThreshold->setValue(10);
}

void MainWindow::on_goHomeButton_clicked()
{
    ui->panSlider->setValue(TrackingCommon::homePan);
    ui->tiltSlider->setValue(TrackingCommon::homeTilt);
    ui->zoomSlider->setValue(TrackingCommon::homeZoom);
    qDebug() << "Zoom has value " << TrackingCommon::homeZoom;
    ui->speedDial->setValue(TrackingCommon::homeSpeed);
    ui->deltaPanTiltSlider->setValue(TrackingCommon::deltaPanTilt);
    ui->cameraCenterThreshold->setValue(TrackingCommon::deltaXYThreshold);

    TrackingCommon::pTZCameraController.setPanTilt(TrackingCommon::homePan, TrackingCommon::homeTilt);
    TrackingCommon::pTZCameraController.setZoom(TrackingCommon::homeZoom);
    TrackingCommon::pTZCameraController.setPanTiltSpeed(TrackingCommon::homeSpeed, TrackingCommon::homeSpeed);
    TrackingCommon::pTZCameraController.setDeltaPanTilt(TrackingCommon::deltaPanTilt, TrackingCommon::deltaPanTilt);
    TrackingCommon::pTZCameraController.setDeltaXYThreshold(TrackingCommon::deltaXYThreshold);

    if (TrackingCommon::positionEstimatorController.isReady() && TrackingCommon::calibrated == true) {
        TrackingCommon::positionEstimatorController.updateTargetHomography(TrackingCommon::pTZCameraController.pan, TrackingCommon::pTZCameraController.tilt, true, true);
    }
}


void MainWindow::on_enable3DReconstructionCheckBox_toggled(bool checked)
{
    TrackingCommon::show3DReconstruction = checked;
}

void MainWindow::on_deltaPanTiltSlider_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setDeltaPanTilt(value, value);
}

void MainWindow::on_centerCameraCheckBox_toggled(bool checked)
{
    TrackingCommon::centerCamera = checked;
}

void MainWindow::on_cameraCenterThreshold_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setDeltaXYThreshold(value);
}

void MainWindow::on_findHomographyRadio_toggled(bool checked)
{
    if (checked)
    {
        TrackingCommon::programMode = TrackingCommon::HOMOGRAPHY_DISCOVERY;
    }
}

void MainWindow::on_saveHomographyButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.saveHomography(fileName.toStdString());
        ui->loadedHomographyLabel->setText(fileName);
    }
}

void MainWindow::on_loadHomographyButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.loadHomography(fileName.toStdString());
        ui->viewHomographyButton->setEnabled(true);
        ui->clearHomographyButton->setEnabled(true);
        ui->loadedHomographyLabel->setText(fileName);
    }
}

void MainWindow::on_scaleHomographyRadio_toggled(bool checked)
{
    if (checked)
        TrackingCommon::scalingMode = TrackingCommon::SCALE_WITH_HOMOGRAPHY;
}

void MainWindow::on_meanShiftAutoScale_toggled(bool checked)
{
    if (checked)
        TrackingCommon::scalingMode = TrackingCommon::SCALE_WITH_MODEL;
}

void MainWindow::on_displayESMParametersCheckbox_toggled(bool checked)
{
    TrackingCommon::showLKParameters = checked;
}

void MainWindow::on_displayIntermediumImagesESMCheck_toggled(bool checked)
{
    TrackingCommon::showLKIntermediumImages = checked;
}

void MainWindow::on_pauseOnEachFrameCheckBox_toggled(bool checked)
{
    TrackingCommon::pauseOnEachFrame = checked;
}

void MainWindow::enable3DReconstruction(bool enable)
{
    ui->enable3DReconstructionCheckBox->setChecked(enable);
}

void MainWindow::enableSaveHomography(bool enable)
{
    ui->saveHomographyButton->setEnabled(enable);
}

void MainWindow::enableMultiFaceTracking(bool enable)
{
    ui->multiFaceTrackingCheckBox->setChecked(enable);
}

void MainWindow::on_viewHomographyButton_clicked()
{
    mainController.displayCurrentHomography();
}

void MainWindow::on_clearHomographyButton_clicked()
{
    mainController.clearCurrentHomography();
    ui->saveHomographyButton->setEnabled(false);
    ui->viewHomographyButton->setEnabled(false);
    ui->loadedHomographyLabel->setText(tr(""));
}

void MainWindow::on_runCalibrationButton_clicked()
{
    char **str;
    str = new char*[12];
    str[0] = "calibration";
    str[1] = "-w"; str[2] = "6";
    str[3] = "-h"; str[4] = "8";
    str[5] = "-s"; str[6] = "2";
    str[7] = "-n"; str[8] = "25";
    str[9] = "-o"; str[10] = "camera.yml";
    str[11] = "-op";
    str[12] = "-oe";
    doCalibration(12, str, TrackingCommon::cameraMatrix, TrackingCommon::distCoeffs);
    TrackingCommon::positionEstimatorController.setCameraIntrinsicParameters(TrackingCommon::cameraMatrix);

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.saveCalibration(fileName.toStdString());
        ui->loadedCalibrationLabel->setText(fileName);
    }

}

void MainWindow::on_loadCalibrationButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.loadCalibration(fileName.toStdString());
        ui->viewCalibrationButton->setEnabled(true);
        ui->clearCalibrationButton->setEnabled(true);
        ui->loadedCalibrationLabel->setText(fileName);
    }
}

void MainWindow::on_viewCalibrationButton_clicked()
{
    mainController.displayCurrentCalibration();
}

void MainWindow::on_clearCalibrationButton_clicked()
{
    mainController.clearCurrentCalibration();
    ui->viewCalibrationButton->setEnabled(false);
    ui->loadedCalibrationLabel->setText(tr(""));
}

void MainWindow::on_glDelaySlider_valueChanged(int value)
{
    TrackingCommon::glDelay = value;
}

void MainWindow::on_showMeanShiftModel_toggled(bool checked)
{
    TrackingCommon::showMeanShiftModel = checked;
}

void MainWindow::on_enableMSModelUpdate_toggled(bool checked)
{
    TrackingCommon::enableMeanShiftModelUpdate = checked;
}

void MainWindow::on_delay1sInEachIteration_toggled(bool checked)
{
    TrackingCommon::delayTracker1sInEachIteration = checked;
}

void MainWindow::on_meanShiftScaleCheckbox_clicked(bool checked)
{
    qDebug() << "Not implemented";
}

void MainWindow::on_recordPositionEstimation_clicked(bool checked)
{
    TrackingCommon::recordPositionEstimation = checked;
}

void MainWindow::on_scalingHomographyCheckBox_clicked(bool checked)
{
    TrackingCommon::enableScalingWithHomography = checked;
}

void MainWindow::on_heightCheckBox_clicked(bool checked)
{
    TrackingCommon::enableHeightCorrection = checked;
}

void MainWindow::on_heightLineEdit_textChanged(const QString &arg1)
{
    TrackingCommon::objectHeight = arg1.toFloat();
}

void MainWindow::on_interpolationCheckBox_clicked(bool checked)
{
    TrackingCommon::enableHomographyInterpolation = checked;
}

void MainWindow::on_filterMeanShift_clicked(bool checked)
{
    TrackingCommon::enableFilteringMeanShift = checked;
    TrackingCommon::enableFilteringMeanShiftToggled = true;
}

void MainWindow::on_enable3DTrackerFiltering_clicked(bool checked)
{
    TrackingCommon::enableFiltering3DPose = checked;
    TrackingCommon::enableFiltering3DPoseToggled = true;
}

void MainWindow::on_displayUnscaledCheckBox_toggled(bool checked)
{
    TrackingCommon::displayUnscaled = checked;
}

void MainWindow::on_selfRecoverCheckBox_toggled(bool checked)
{
    TrackingCommon::selfRecover = checked;
}

void MainWindow::on_newPlanarModelButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    QString(),
                                                    tr("Images (*.jpg *.jpeg *.png *.bmp)"));
    if (fileName != "")
    {
        mainController.addPlaneModel(fileName.toStdString());

        // Refresh all items in list.... Later this code will be moved
        ui->modelsList->clear();
        for (int i = 0; i < TrackingCommon::objectModels.size(); ++i)
        {
            std::string title = ((ObjectModel*)TrackingCommon::objectModels.at(i))->getTitle();
            ui->modelsList->addItem(title.c_str());
        }
        if (TrackingCommon::objectModels.size() > 0)
        {
            ui->findModelButton->setChecked(true);
        }
    }
}

void MainWindow::on_viewModelButton_clicked()
{
    int seletedIndex = ui->modelsList->currentRow();
    if (seletedIndex >= 0)
    {
        FeatureBasedModel* model = ((FeatureBasedModel*)TrackingCommon::objectModels.at(seletedIndex));
        cv::Mat modelTemplate = ((cv::Mat) model->templates.at(0)).clone();
        TrackingCommon::highlightKeypoints(modelTemplate, model->keypoints);
        cv::imshow(model->getTitle(), modelTemplate);
    }
}

void MainWindow::on_findModelButton_toggled(bool checked)
{
    TrackingCommon::findModels = checked;
}

void MainWindow::on_enableMotionFilterCheckBox_stateChanged(int arg1)
{
}

void MainWindow::on_enableMotionFilterCheckBox_toggled(bool checked)
{
    TrackingCommon::enableMotionFilter = checked;
}

void MainWindow::on_linePlaneSearchCheckBox_toggled(bool checked)
{
    TrackingCommon::searchPlaneLines = checked;
}

void MainWindow::on_markReferenceButton_clicked()
{
    TrackingCommon::markObjectZero = true;
    ui->saveReferenceButton->setEnabled(true);
}

void MainWindow::on_linesWidthLineEdit_textChanged(const QString &arg1)
{
    TrackingCommon::linesWidth = arg1.toFloat();
}

void MainWindow::on_linesHeightLineEdit_textChanged(const QString &arg1)
{
    TrackingCommon::linesHeight = arg1.toFloat();
}

void MainWindow::on_lineWidthLineEdit_textChanged(const QString &arg1)
{
    TrackingCommon::lineWidth = arg1.toFloat();
}

void MainWindow::on_markRotReferenceButton_clicked()
{
    TrackingCommon::markObjectRotZero = true;
    ui->saveReferenceButton->setEnabled(true);
}

void MainWindow::on_gpSerialPortLineEdit_textChanged(const QString &arg1)
{
    TrackingCommon::gpSerialPortName = arg1.toStdString();
}

void MainWindow::on_gppSerialPortButton_clicked()
{
    if (!TrackingCommon::gpSerialPort->isOpen())
    {
        connectGPSerialPort();
    }
    else
    {
        disconnectGPSerialPort();
    }
    qDebug("GP Serial Port Status: %d", TrackingCommon::gpSerialPort->isOpen());
}

void MainWindow::on_showLinesPoseCheckBox_toggled(bool checked)
{
    TrackingCommon::searchPlaneLinesRecord = checked;
}

void MainWindow::on_saveReferenceButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.saveCameraReference(fileName.toStdString());
    }

}

void MainWindow::on_loadReferenceButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    QString(),
                                                    tr("OpenCV Yml (*.yml)"));
    if (fileName != "")
    {
        mainController.loadHomography(fileName.toStdString());
    }
}

void MainWindow::on_colorBrightnessThresholdSlider_valueChanged(int value)
{
    TrackingCommon::colorBrightnessThreshold = value;
}

void MainWindow::on_areaThresholdSlider_valueChanged(int value)
{
    TrackingCommon::areaThreshold = value;
}

void MainWindow::on_colorHSVThresholdSlider_valueChanged(int value)
{
    TrackingCommon::colorHSVThreshold = value;
}

void MainWindow::on_focusSlider_valueChanged(int value)
{
    TrackingCommon::pTZCameraController.setFocus(value);
}

void MainWindow::on_autoFocusCheckBox_toggled(bool checked)
{
    TrackingCommon::pTZCameraController.setFocusAutoOn(checked);
}

void MainWindow::on_brightnessUpButton_clicked()
{
    TrackingCommon::pTZCameraController.setBrightnessUp();
}

void MainWindow::on_brightnessDownButton_clicked()
{
    TrackingCommon::pTZCameraController.setBrightnessDown();
}

void MainWindow::displayParrotControlValues(int roll, int pitch, int yaw, int verticalSpeed)
{
    std::string positiveStyle = "QLabel { background-color : green; color : white; }";
    std::string neutralStyle = "QLabel { background-color : white; color : black; }";
    std::string negativeStyle = "QLabel { background-color : red; color : white; }";
    ui->rollValueLabel->setText(QString::number(roll));
    if (roll == 0)
        ui->rollValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (roll > 0)
        ui->rollValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->rollValueLabel->setStyleSheet(negativeStyle.c_str());

    ui->pitchValueLabel->setText(QString::number(pitch));
    if (pitch == 0)
        ui->pitchValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (pitch > 0)
        ui->pitchValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->pitchValueLabel->setStyleSheet(negativeStyle.c_str());

    ui->yawValueLabel->setText(QString::number(yaw));
    if (yaw == 0)
        ui->yawValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (yaw > 0)
        ui->yawValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->yawValueLabel->setStyleSheet(negativeStyle.c_str());

    ui->heightValueLabel->setText(QString::number(verticalSpeed));
    if (verticalSpeed == 0)
        ui->heightValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (verticalSpeed > 0)
        ui->heightValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->heightValueLabel->setStyleSheet(negativeStyle.c_str());
}

void MainWindow::displayHapticInputValues(int x, int y)
{
    std::string positiveStyle = "QLabel { background-color : green; color : white; }";
    std::string neutralStyle = "QLabel { background-color : white; color : black; }";
    std::string negativeStyle = "QLabel { background-color : red; color : white; }";
    ui->hapticXValueLabel->setText(QString::number(x));
    if (x == 0)
        ui->hapticXValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (x > 0)
        ui->hapticXValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->hapticXValueLabel->setStyleSheet(negativeStyle.c_str());

    ui->hapticYValueLabel->setText(QString::number(y));
    if (y == 0)
        ui->hapticYValueLabel->setStyleSheet(neutralStyle.c_str());
    else if (y > 0)
        ui->hapticYValueLabel->setStyleSheet(positiveStyle.c_str());
    else
        ui->hapticYValueLabel->setStyleSheet(negativeStyle.c_str());
}

void MainWindow::displayBatteryCharge(int batteryCharge)
{
    ui->batteryBar->setValue(batteryCharge);
}

void MainWindow::setParrotAutomaticControl(bool automaticControl)
{
    ui->parrotAutomaticControlButton->setChecked(automaticControl);
}

void MainWindow::setParrotConnectionStatus(bool connected)
{
    ui->parrotEmergencyStopButton->setChecked(connected);
}

void MainWindow::updateParrotTrajectoryStatus(int point, bool completed)
{
    QTableWidgetItem* checkedWidget = ui->parrotPointsTable->item(point, 0);
    checkedWidget->setCheckState(completed ? Qt::Checked : Qt::Unchecked);

    for (int i = 0; i < ui->parrotPointsTable->rowCount(); ++i)
    {
        QTableWidgetItem* checkedWidget = ui->parrotPointsTable->item(i, 0);
        if (!TrackingCommon::parrotTrajectory.empty())
            TrackingCommon::parrotTrajectory.at<float>(i, 3) = checkedWidget->checkState() == Qt::Checked ? 1.0 : -1.0;
    }
}

void MainWindow::create3DReconstructionWindow()
{
    qDebug() << ">> Creating GlWindow";
//    MainWindowTemp mainWin;
//    mainWin.resize(600, 400);
//    mainWin.show();
//    GLDrawingWindow *drawingWindow = new GLDrawingWindow();
//    drawingWindow->show();
}

void MainWindow::displayCVImage(std::string windowTitle, cv::Mat image)
{
    cv::imshow(windowTitle, image);
}

void MainWindow::on_autoBrightnessButton_clicked(bool checked)
{
    if (checked)
    {
        TrackingCommon::pTZCameraController.setBrightnessAutoOn();
    }
    else
    {
        TrackingCommon::pTZCameraController.setBrightnessAutoOff();
    }
}

void MainWindow::on_cuboidSelectedFaceComboBox_currentIndexChanged(int index)
{
    TrackingCommon::cuboidSelectedFace = index;
}

void MainWindow::on_cuboidModelsComboBox_currentIndexChanged(int index)
{
    TrackingCommon::cuboidSelectedModel = index;
}

void MainWindow::on_areaMaxThresholdSlider_valueChanged(int value)
{
    TrackingCommon::areaMaxThreshold = value;
}

void MainWindow::on_parrotTakeOffButton_clicked()
{
    TrackingCommon::mainController->takeOffParrot();
}

void MainWindow::on_reconstructionSetZero_clicked()
{
    TrackingCommon::markObjectZero = true;
}

void MainWindow::on_parrotEmergencyStopButton_toggled(bool checked)
{
    QIcon icon;
    mainController.connectJoypad(0);
    if (TrackingCommon::joypadController.isRunning())
    {
        if (!checked)
        {
            qDebug() << "Disconnecting parrot";
            icon.addFile(QString::fromUtf8("../start.png"), QSize(), QIcon::Normal, QIcon::Off);
            ui->parrotEmergencyStopButton->setIcon(icon);
            ui->parrotEmergencyStopButton->setText("Start");
            ui->parrotTakeOffButton->setEnabled(false);
            ui->parrotAutomaticControlButton->setEnabled(false);
            TrackingCommon::mainController->landParrot();
            TrackingCommon::mainController->disconnectParrot();
        }
        else
        {
            qDebug() << "Connecting parrot";
            icon.addFile(QString::fromUtf8("../stop.png"), QSize(), QIcon::Normal, QIcon::Off);
            ui->parrotEmergencyStopButton->setIcon(icon);
            ui->parrotEmergencyStopButton->setText("Stop");
            ui->parrotTakeOffButton->setEnabled(true);
            ui->parrotAutomaticControlButton->setEnabled(true);
            TrackingCommon::mainController->connectParrot();
        }

    }
    else
    {
        if (checked)
        {
            qDebug() << "! Please connect Joystick";
            ui->parrotEmergencyStopButton->setChecked(false);
        }
    }
}

void MainWindow::on_parrotAutomaticControlButton_toggled(bool checked)
{
    TrackingCommon::parrotController.setAutomaticControl(checked);
    if (checked) {
        (Phonon::createPlayer(Phonon::MusicCategory, Phonon::MediaSource("../control-automatico.wav")))->play();
    } else {
        (Phonon::createPlayer(Phonon::MusicCategory, Phonon::MediaSource("../control-manual.wav")))->play();
    }
}

void MainWindow::on_parrotAddPoint_clicked()
{
    int currentRow = ui->parrotPointsTable->rowCount();
    ui->parrotPointsTable->setRowCount(currentRow + 1);

    QTableWidgetItem* defaultValue = new QTableWidgetItem();
    defaultValue->setCheckState(Qt::Unchecked);
    ui->parrotPointsTable->setItem(currentRow, 0, defaultValue);

    for (int i = 1; i < 5; ++i)
    {
        QTableWidgetItem* defaultValue = new QTableWidgetItem(QString::number(0, 'f', 2));
        ui->parrotPointsTable->setItem(currentRow, i, defaultValue);
    }
}

void MainWindow::on_parrotExecuteTrajectory_clicked()
{
    cv::Mat trajectory = cv::Mat(ui->parrotPointsTable->rowCount(), 4, CV_32F);
    for (int i = 0; i < ui->parrotPointsTable->rowCount(); ++i)
    {
        QTableWidgetItem* checkedWidget = ui->parrotPointsTable->item(i, 0);
        checkedWidget->setCheckState(Qt::Unchecked);
        QTableWidgetItem* xWidget = ui->parrotPointsTable->item(i, 1);
        QTableWidgetItem* yWidget = ui->parrotPointsTable->item(i, 2);
        QTableWidgetItem* zWidget = ui->parrotPointsTable->item(i, 3);
        QTableWidgetItem* yawWidget = ui->parrotPointsTable->item(i, 4);
        if (xWidget && yWidget && zWidget && yawWidget)
        {
            float x = xWidget->text().toFloat();
            float y = yWidget->text().toFloat();
            float z = zWidget->text().toFloat();
            float yaw = yawWidget->text().toFloat();
            trajectory.at<float>(i, 0) = x;
            trajectory.at<float>(i, 1) = y;
            trajectory.at<float>(i, 2) = z;
            trajectory.at<float>(i, 3) = yaw;
        }
    }

    TrackingCommon::mainController->setTrajectory(trajectory);
}

void MainWindow::on_interlaceCheckBox_toggled(bool checked)
{
    TrackingCommon::enableDeinterlace = checked;
}

void MainWindow::on_sendPositionDataCheckBox_toggled(bool checked)
{
    TrackingCommon::enableSendingPositionData = checked;
}

void MainWindow::on_cuboidTrackingRadio_toggled(bool checked)
{
    if (checked)
    {
        TrackingCommon::trackingMode = TrackingCommon::TRACKING_CUBOID;
        TrackingCommon::programMode = TrackingCommon::TRACKING;
    }
}

void MainWindow::on_undistortCheckBox_toggled(bool checked)
{
    TrackingCommon::enableUndistort = checked;
}

void MainWindow::on_lockFaceCheckBox_toggled(bool checked)
{
    TrackingCommon::enableLockFace = checked;
    mainController.setFaceLock(checked);
}

void MainWindow::on_multiFaceTrackingCheckBox_toggled(bool checked)
{
    TrackingCommon::enableMultipleFaceTracking = checked;
}

void MainWindow::on_parrotSavePathButton_clicked()
{
    cv::Mat pathTable = cv::Mat (ui->parrotPointsTable->rowCount(), ui->parrotPointsTable->columnCount()-1, CV_32F);
    for (int i = 0; i < ui->parrotPointsTable->rowCount(); ++i)
    {
        for (int j = 1; j < ui->parrotPointsTable->columnCount(); ++j)
        {
            pathTable.at<float>(i,j-1) = ui->parrotPointsTable->item(i,j)->text().toFloat();
        }
    }

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),QString(),tr("OpenCV Yml (*.yml)"));

    if (fileName != "")
    {
        cv::FileStorage fileStorage = cv::FileStorage(fileName.toStdString().c_str(), CV_STORAGE_WRITE);
        fileStorage << "PathTable" << pathTable;
        fileStorage.release();
        qDebug()<<"saved path " << fileName;
    }
}

void MainWindow::on_parrotLoadButton_clicked()
{

//    cv::Mat rotation  = (cv::Mat_<float>(3, 3) << 0.73267608	,0.00311023	,0.711688 ,-0.0241035	,0.999521	,0.0194238 ,-0.711286	,-0.0307992	,0.702228);
//    cv::Mat angles = cv::Mat(3, 1, CV_32F);
//    cv::Rodrigues(rotation, angles);
//    double toDeg = 180.0 / CV_PI;
//    angles *= toDeg;
//    mainController.displayData(std::string("Angles"), angles, DataDisplayer::APPEND);

    cv::Mat pathTable;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),QString(),tr("Path File (*)"));

    if (fileName != "")
    {
        cv::FileStorage fileStorage = cv::FileStorage(fileName.toStdString().c_str(), CV_STORAGE_READ);
        if (fileStorage.isOpened())
        {
            fileStorage["PathTable"] >> pathTable;
            fileStorage.release();
        }
    }

    int totalRows = pathTable.rows;
    int totalColumns = pathTable.cols + 1;
    ui->parrotPointsTable->setRowCount(totalRows);
    ui->parrotPointsTable->setColumnCount(totalColumns);

    for (int i = 0; i < totalRows + 1; ++i)
    {
        QTableWidgetItem* defaultValue = new QTableWidgetItem();
        defaultValue->setCheckState(Qt::Unchecked);
        ui->parrotPointsTable->setItem(i, 0, defaultValue);
    }

    for (int i = 1; i < totalColumns; ++i)
    {
        for (int j = 0; j < totalRows; ++j)
        {
            QTableWidgetItem* defaultValue = new QTableWidgetItem(QString::number(pathTable.at<float>(j, i - 1), 'f', 2));
            ui->parrotPointsTable->setItem(j, i, defaultValue);
        }
    }
}

void MainWindow::on_calibrationRefreshButton_clicked()
{
    ui->calibrationTable->setRowCount(3);
    ui->calibrationTable->setColumnCount(3);
    for (int i = 0; i < 3; ++i)
    {
        ui->calibrationTable->setColumnWidth(i, 65);
        for (int j = 0; j < 3; ++j)
        {
            QTableWidgetItem* defaultValue = new QTableWidgetItem(QString::number(TrackingCommon::cameraMatrix.at<double>(i, j)));
            ui->calibrationTable->setItem(i, j, defaultValue);
        }
    }
}

void MainWindow::on_calibrationApplyButton_clicked()
{
    if (ui->calibrationTable->rowCount() > 0) {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                TrackingCommon::cameraMatrix.at<double>(i, j) = ui->calibrationTable->item(i, j)->text().toDouble();
            }
        }
    }
}

void MainWindow::on_applyControlBarsButton_clicked()
{
    mainController.setParrotControlVariables(
                ui->kpRollPitch->text().toFloat(), ui->kpRollPitchSpeed->text().toFloat(), ui->kpYaw->text().toFloat(), ui->kpVerticalSpeed->text().toFloat(),
                ui->kdRollPitch->text().toFloat(), ui->kdRollPitchSpeed->text().toFloat(), ui->kdYaw->text().toFloat(), ui->kdVerticalSpeed->text().toFloat(),
                ui->kiRollPitch->text().toFloat(), ui->kiRollPitchSpeed->text().toFloat(), ui->kiYaw->text().toFloat(), ui->kiVerticalSpeed->text().toFloat(),
                ui->parrotFilterInput->text().toFloat(), ui->parrotFilterOutput->text().toFloat(), ui->parrotMaxSpeed->text().toFloat()
                );
}

void MainWindow::on_controlVarsSaveButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), QString(), tr("OpenCV Yml (*.yml)"));

    if (fileName != "")
    {
        cv::FileStorage fileStorage = cv::FileStorage(fileName.toStdString().c_str(), CV_STORAGE_WRITE);

        fileStorage << "kpRollPitch" << ui->kpRollPitch->text().toFloat();
        fileStorage << "kpRollPitchSpeed" << ui->kpRollPitchSpeed->text().toFloat();
        fileStorage << "kpYaw" << ui->kpYaw->text().toFloat();
        fileStorage << "kpVerticalSpeed" << ui->kpVerticalSpeed->text().toFloat();

        fileStorage << "kdRollPitch" << ui->kdRollPitch->text().toFloat();
        fileStorage << "kdRollPitchSpeed" << ui->kdRollPitchSpeed->text().toFloat();
        fileStorage << "kdYaw" << ui->kdYaw->text().toFloat();
        fileStorage << "kdVerticalSpeed" << ui->kdVerticalSpeed->text().toFloat();

        fileStorage << "kiRollPitch" << ui->kiRollPitch->text().toFloat();
        fileStorage << "kiRollPitchSpeed" << ui->kiRollPitchSpeed->text().toFloat();
        fileStorage << "kiYaw" << ui->kiYaw->text().toFloat();
        fileStorage << "kiVerticalSpeed" << ui->kiVerticalSpeed->text().toFloat();

        fileStorage << "inputFilterScale" << ui->parrotFilterInput->text().toFloat();
        fileStorage << "outputFilterScale" << ui->parrotFilterOutput->text().toFloat();
        fileStorage << "maxRollPitchSpeed" << ui->parrotMaxSpeed->text().toFloat();

        fileStorage.release();
        qDebug()<<"Control vars saved at " << fileName;
    }

}

void MainWindow::loadParrotControlVariables(std::string fileName)
{
    if (fileName != "")
    {
        cv::FileStorage fileStorage = cv::FileStorage(fileName.c_str(), CV_STORAGE_READ);
        if (fileStorage.isOpened())
        {
            float kpRollPitch, kpRollPitchSpeed, kpYaw, kpVerticalSpeed,
                  kdRollPitch, kdRollPitchSpeed, kdYaw, kdVerticalSpeed,
                  kiRollPitch, kiRollPitchSpeed, kiYaw, kiVerticalSpeed,
                  inputFilterScale, outputFilterScale, maxRollPitchSpeed;

            fileStorage["kpRollPitch"] >> kpRollPitch;
            fileStorage["kpRollPitchSpeed"] >> kpRollPitchSpeed;
            fileStorage["kpYaw"] >> kpYaw;
            fileStorage["kpVerticalSpeed"] >> kpVerticalSpeed;

            fileStorage["kdRollPitch"] >> kdRollPitch;
            fileStorage["kdRollPitchSpeed"] >> kdRollPitchSpeed;
            fileStorage["kdYaw"] >> kdYaw;
            fileStorage["kdVerticalSpeed"] >> kdVerticalSpeed;

            fileStorage["kiRollPitch"] >> kiRollPitch;
            fileStorage["kiRollPitchSpeed"] >> kiRollPitchSpeed;
            fileStorage["kiYaw"] >> kiYaw;
            fileStorage["kiVerticalSpeed"] >> kiVerticalSpeed;

            fileStorage["inputFilterScale"] >> inputFilterScale;
            fileStorage["outputFilterScale"] >> outputFilterScale;
            fileStorage["maxRollPitchSpeed"] >> maxRollPitchSpeed;

            fileStorage.release();

            ui->kpRollPitch->setText(QString::number(kpRollPitch));
            ui->kpRollPitchSpeed->setText(QString::number(kpRollPitchSpeed));
            ui->kpYaw->setText(QString::number(kpYaw));
            ui->kpVerticalSpeed->setText(QString::number(kpVerticalSpeed));

            ui->kdRollPitch->setText(QString::number(kdRollPitch));
            ui->kdRollPitchSpeed->setText(QString::number(kdRollPitchSpeed));
            ui->kdYaw->setText(QString::number(kdYaw));
            ui->kdVerticalSpeed->setText(QString::number(kdVerticalSpeed));

            ui->kiRollPitch->setText(QString::number(kiRollPitch));
            ui->kiRollPitchSpeed->setText(QString::number(kiRollPitchSpeed));
            ui->kiYaw->setText(QString::number(kiYaw));
            ui->kiVerticalSpeed->setText(QString::number(kiVerticalSpeed));

            ui->parrotFilterInput->setText(QString::number(inputFilterScale));
            ui->parrotFilterOutput->setText(QString::number(outputFilterScale));
            ui->parrotMaxSpeed->setText(QString::number(maxRollPitchSpeed));
        }
    }
}

void MainWindow::on_controlVarsLoadButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),QString(), tr("OpenCV (*.yml)"));

    if (fileName != "")
    {
        loadParrotControlVariables(fileName.toStdString());
        ui->parrotControlVariablesFilename->setText(fileName);
    }
}

void MainWindow::on_maxTrackedFaces_textChanged(const QString &arg1)
{
    TrackingCommon::maxTrackedFaces = arg1.toInt();
}

void MainWindow::on_socketsStartButton_clicked(bool checked)
{
    if (checked) {
        TrackingCommon::mainSocket.initSocket(std::string("10.20.219.211"), 12345);
        ui->socketsStartButton->setText(QString("Disconnect"));
    } else {
        TrackingCommon::mainSocket.stopSocket();
        ui->socketsStartButton->setText(QString("Connect"));
    }
}

void MainWindow::on_parrotDisplayPathButton_clicked()
{
}

void MainWindow::on_enableHapticInput_toggled(bool checked)
{
    TrackingCommon::enableHapticControl = checked;
    if (checked) {
        ui->enableHapticInput->setText(QString("Haptic On"));
    } else {
        ui->enableHapticInput->setText(QString("Haptic Off"));
    }
}

void MainWindow::on_parrotDisplayPathButton_toggled(bool checked)
{
    if (checked) {
        TrackingCommon::parrotTrajectory = cv::Mat(ui->parrotPointsTable->rowCount(), 4, CV_32F);
        for (int i = 0; i < ui->parrotPointsTable->rowCount(); ++i)
        {
            QTableWidgetItem* checkedWidget = ui->parrotPointsTable->item(i, 0);
            QTableWidgetItem* xWidget = ui->parrotPointsTable->item(i, 1);
            QTableWidgetItem* yWidget = ui->parrotPointsTable->item(i, 2);
            QTableWidgetItem* zWidget = ui->parrotPointsTable->item(i, 3);
            if (xWidget && yWidget && zWidget)
            {
                float x = xWidget->text().toFloat();
                float y = yWidget->text().toFloat();
                float z = zWidget->text().toFloat();
                TrackingCommon::parrotTrajectory.at<float>(i, 0) = x;
                TrackingCommon::parrotTrajectory.at<float>(i, 1) = y;
                TrackingCommon::parrotTrajectory.at<float>(i, 2) = z;
                TrackingCommon::parrotTrajectory.at<float>(i, 3) = checkedWidget->checkState() == Qt::Checked ? 1.0 : -1.0;
            }
        }
        QStringList arguments;
        arguments << "--persist --interactive";
        QProcess octave;
        octave.setProcessChannelMode(QProcess::MergedChannels);
        octave.start("/usr/bin/octave3.2", arguments);
    }
    TrackingCommon::enablePathDrawing = checked;
}
