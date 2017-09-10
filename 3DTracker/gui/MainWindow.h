#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "opencv2/core/core.hpp"
#include "common/MainController.h"
#include "gui/DataDisplayer.h"
#include "gui/WindowCallbacks.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent *);
    MainController mainController;

    void populateTestsCombobox();
    void connectSerialPort();
    void disconnectSerialPort();
    void connectGPSerialPort();
    void disconnectGPSerialPort();
    void showEvent(QShowEvent *);
    void loadParrotControlVariables(std::string fileName);

private slots:
    void updateImageWindow(std::string windowTitle, const QImage &image);
    void destroyImageWindows();
    void setImageWindowsMouseCallBack(std::string windowTitle, WindowCallBack* windowCallBack);
    void displayData(std::string tabName, cv::Mat mat, DataDisplayer::DataDisplayModes dataDisplayMode);
    void displayCoordinates(std::string tabName, std::string displayName, cv::Mat displayData);
    void displayPose(std::string tabName, std::string displayName, cv::Mat displayData);
    void enable3DReconstruction(bool enable);
    void enableSaveHomography(bool enable);
    void enableMultiFaceTracking(bool enable);
    void displayParrotControlValues(int roll, int pitch, int yaw, int verticalSpeed);
    void displayHapticInputValues(int x, int y);
    void setParrotAutomaticControl(bool automaticControl);
    void setParrotConnectionStatus(bool connected);
    void updateParrotTrajectoryStatus(int point, bool completed);
    void create3DReconstructionWindow();
    void displayCVImage(std::string windowTitle, cv::Mat image);
    void displayBatteryCharge(int batteryCharge);

    void on_startVideoButton_clicked();

    void on_stopVideoButton_clicked();

    void on_cameraLineEdit_textChanged(const QString &arg1);

    void on_selectMediaButton_clicked();

    void on_cameraRadio_clicked(bool checked);

    void on_mediaRadio_clicked(bool checked);

    void on_launchTestButton_clicked();

    void on_pauseOnFirstFrameCheckBox_clicked(bool checked);

    void on_pauseVideoButton_clicked();

    void on_ESMRadio_clicked(bool checked);

    void on_meanShiftRadio_clicked(bool checked);

    void on_LKAdditiveRadio_clicked(bool checked);

    void on_LKInverseRadio_clicked(bool checked);

    void on_zoomSlider_valueChanged(int value);

    void on_serialPortConnectButton_clicked();

    void on_panSlider_valueChanged(int value);

    void on_tiltSlider_valueChanged(int value);

    void on_speedDial_valueChanged(int value);

    void on_cameraSetHomeButton_clicked();

    void on_cameraResetButton_clicked();

    void on_goHomeButton_clicked();

    void on_enable3DReconstructionCheckBox_toggled(bool checked);

    void on_deltaPanTiltSlider_valueChanged(int value);

    void on_centerCameraCheckBox_toggled(bool checked);

    void on_cameraCenterThreshold_valueChanged(int value);

    void on_findHomographyRadio_toggled(bool checked);

    void on_saveHomographyButton_clicked();

    void on_loadHomographyButton_clicked();

    void on_scaleHomographyRadio_toggled(bool checked);

    void on_meanShiftAutoScale_toggled(bool checked);

    void on_displayESMParametersCheckbox_toggled(bool checked);

    void on_displayIntermediumImagesESMCheck_toggled(bool checked);

    void on_pauseOnEachFrameCheckBox_toggled(bool checked);

    void on_viewHomographyButton_clicked();

    void on_clearHomographyButton_clicked();

    void on_runCalibrationButton_clicked();

    void on_loadCalibrationButton_clicked();

    void on_viewCalibrationButton_clicked();

    void on_clearCalibrationButton_clicked();

    void on_glDelaySlider_valueChanged(int value);

    void on_showMeanShiftModel_toggled(bool checked);

    void on_enableMSModelUpdate_toggled(bool checked);

    void on_delay1sInEachIteration_toggled(bool checked);

    void on_meanShiftScaleCheckbox_clicked(bool checked);

    void on_recordPositionEstimation_clicked(bool checked);

    void on_scalingHomographyCheckBox_clicked(bool checked);

    void on_heightCheckBox_clicked(bool checked);

    void on_heightLineEdit_textChanged(const QString &arg1);

    void on_interpolationCheckBox_clicked(bool checked);

    void on_filterMeanShift_clicked(bool checked);

    void on_enable3DTrackerFiltering_clicked(bool checked);

    void on_displayUnscaledCheckBox_toggled(bool checked);

    void on_selfRecoverCheckBox_toggled(bool checked);

    void on_newPlanarModelButton_clicked();

    void on_viewModelButton_clicked();

    void on_findModelButton_toggled(bool checked);

    void on_enableMotionFilterCheckBox_stateChanged(int arg1);

    void on_enableMotionFilterCheckBox_toggled(bool checked);

    void on_linePlaneSearchCheckBox_toggled(bool checked);

    void on_markReferenceButton_clicked();

    void on_linesWidthLineEdit_textChanged(const QString &arg1);

    void on_linesHeightLineEdit_textChanged(const QString &arg1);

    void on_lineWidthLineEdit_textChanged(const QString &arg1);

    void on_markRotReferenceButton_clicked();

    void on_gpSerialPortLineEdit_textChanged(const QString &arg1);

    void on_gppSerialPortButton_clicked();

    void on_showLinesPoseCheckBox_toggled(bool checked);

    void on_saveReferenceButton_clicked();

    void on_loadReferenceButton_clicked();

    void on_colorBrightnessThresholdSlider_valueChanged(int value);

    void on_areaThresholdSlider_valueChanged(int value);

    void on_colorHSVThresholdSlider_valueChanged(int value);

    void on_focusSlider_valueChanged(int value);

    void on_autoFocusCheckBox_toggled(bool checked);

    void on_brightnessUpButton_clicked();

    void on_brightnessDownButton_clicked();

    void on_autoBrightnessButton_clicked(bool checked);

    void on_cuboidSelectedFaceComboBox_currentIndexChanged(int index);

    void on_cuboidModelsComboBox_currentIndexChanged(int index);

    void on_areaMaxThresholdSlider_valueChanged(int value);

    void on_parrotTakeOffButton_clicked();

    void on_reconstructionSetZero_clicked();

    void on_parrotEmergencyStopButton_toggled(bool checked);

    void on_parrotAutomaticControlButton_toggled(bool checked);

    void on_parrotAddPoint_clicked();

    void on_parrotExecuteTrajectory_clicked();

    void on_interlaceCheckBox_toggled(bool checked);

    void on_sendPositionDataCheckBox_toggled(bool checked);

    void on_cuboidTrackingRadio_toggled(bool checked);

    void on_undistortCheckBox_toggled(bool checked);

    void on_lockFaceCheckBox_toggled(bool checked);

    void on_multiFaceTrackingCheckBox_toggled(bool checked);

    void on_parrotSavePathButton_clicked();

    void on_parrotLoadButton_clicked();

    void on_calibrationRefreshButton_clicked();

    void on_calibrationApplyButton_clicked();

    void on_applyControlBarsButton_clicked();

    void on_controlVarsSaveButton_clicked();

    void on_controlVarsLoadButton_clicked();

    void on_maxTrackedFaces_textChanged(const QString &arg1);

    void on_socketsStartButton_clicked(bool checked);

    void on_parrotDisplayPathButton_clicked();

    void on_enableHapticInput_toggled(bool checked);

    void on_parrotDisplayPathButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
