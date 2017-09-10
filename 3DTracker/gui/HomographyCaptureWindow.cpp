#include "gui/HomographyCaptureWindow.h"
#include "ui_HomographyCaptureWindow.h"
#include <QDebug>
#include <QMessageBox>
#include "opencv2/core/core.hpp"
#include "common/TrackingCommon.h"

HomographyCaptureWindow::HomographyCaptureWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HomographyCaptureWindow)
{
    ui->setupUi(this);
    QStringList tableHeader = QStringList();
    tableHeader.push_back("X Img");
    tableHeader.push_back("Y Img");
    tableHeader.push_back("X Real");
    tableHeader.push_back("Y Real");
    ui->discoveryPointTable->setColumnCount(4);
    ui->discoveryPointTable->setHorizontalHeaderLabels(tableHeader);
    ui->discoveryPointTable->setColumnWidth(0, 60);
    ui->discoveryPointTable->setColumnWidth(1, 60);
    ui->discoveryPointTable->setColumnWidth(2, 80);
    ui->discoveryPointTable->setColumnWidth(3, 80);
}

HomographyCaptureWindow::~HomographyCaptureWindow()
{
    delete ui;
}

void HomographyCaptureWindow::addImagePoint(float x, float y)
{
    int currentRow = ui->discoveryPointTable->rowCount();
    ui->discoveryPointTable->setRowCount(currentRow + 1);
    QTableWidgetItem* xImage = new QTableWidgetItem(QString::number(x, 'f', 2));
    ui->discoveryPointTable->setItem(currentRow, 0, xImage);
    QTableWidgetItem* yImage = new QTableWidgetItem(QString::number(y, 'f', 2));
    ui->discoveryPointTable->setItem(currentRow, 1, yImage);
}

void HomographyCaptureWindow::on_deleteHomographyEntry_clicked()
{
    while (ui->discoveryPointTable->selectedItems().count() > 0)
    {
        ui->discoveryPointTable->removeRow(ui->discoveryPointTable->selectedItems().at(0)->row());
    }
}

void HomographyCaptureWindow::on_findHomographyButton_clicked()
{
    std::vector<float> sourcePointsX;
    std::vector<float> sourcePointsY;
    std::vector<float> detinationPointsX;
    std::vector<float> detinationPointsY;
    for (int i = 0; i < ui->discoveryPointTable->rowCount(); ++i)
    {
        QTableWidgetItem* imagePointXWidget = ui->discoveryPointTable->item(i, 0);
        QTableWidgetItem* imagePointYWidget = ui->discoveryPointTable->item(i, 1);
        QTableWidgetItem* realPointXWidget = ui->discoveryPointTable->item(i, 2);
        QTableWidgetItem* realPointYWidget = ui->discoveryPointTable->item(i, 3);
        if (imagePointXWidget && imagePointYWidget && realPointXWidget && realPointYWidget)
        {
            float imagePointX = imagePointXWidget->text().toFloat();
            float imagePointY = imagePointYWidget->text().toFloat();
            float realPointX = realPointXWidget->text().toFloat();
            float realPointY = realPointYWidget->text().toFloat();
            sourcePointsX.push_back(imagePointX);
            sourcePointsY.push_back(imagePointY);
            detinationPointsX.push_back(realPointX);
            detinationPointsY.push_back(realPointY);
        }
    }
    if (sourcePointsX.size() < 4)
    {
        QMessageBox msgBox;
        msgBox.addButton(QMessageBox::Ok);
        msgBox.setText("Please introduce information of at least 4 points");
        msgBox.exec();
        return;
    }
    findHomography(sourcePointsX, sourcePointsY, detinationPointsX, detinationPointsY);
}

void HomographyCaptureWindow::findHomography(std::vector<float> imagePointsX, std::vector<float> imagePointsY, std::vector<float> realPointsX, std::vector<float> realPointsY)
{
    TrackingCommon::positionEstimatorController.positionEstimator.clearTrainingPoints();
    for (int i = 0; i < imagePointsX.size(); ++i)
    {
        cv::Point2f trainingImagePoint = cv::Point2f(imagePointsX[i], imagePointsY[i]);
        cv::Point2f trainingRealPoint = cv::Point2f(realPointsX[i], realPointsY[i]);
        TrackingCommon::positionEstimatorController.positionEstimator.addTrainingPoint(trainingImagePoint, trainingRealPoint);
    }
    TrackingCommon::positionEstimatorController.positionEstimator.findHomography();

    if (TrackingCommon::positionEstimatorController.positionEstimator.isReady)
    {
        TrackingCommon::mainController->enableHomographySave(true);
        if (TrackingCommon::showDiscoveredHomography)
        {
            TrackingCommon::dataDisplayer.displayData("Homography", TrackingCommon::positionEstimatorController.positionEstimator.H, DataDisplayer::FULL);
        }
    }
    else
    {
        qDebug() << "Position Estimator not Ready";
    }
}
