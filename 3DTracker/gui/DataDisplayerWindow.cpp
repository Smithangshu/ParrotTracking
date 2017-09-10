#include "gui/DataDisplayerWindow.h"
#include "ui_DataDisplayerWindow.h"
#include <QVBoxLayout>
#include <QTableWidget>
#include <QStringList>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QScrollArea>
#include <typeinfo>
#include "common/TrackingCommon.h"
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <QDateTime>

DataDisplayerWindow::DataDisplayerWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataDisplayerWindow)
{
    ui->setupUi(this);
}

DataDisplayerWindow::~DataDisplayerWindow()
{
    delete ui;
}

int DataDisplayerWindow::addTab(std::string tabName, bool insertTable)
{
    QWidget* tab = new QWidget;
    QVBoxLayout* mainLayout = new QVBoxLayout;

    if (insertTable)
    {
        QTableWidget* table = new QTableWidget;
        mainLayout->addWidget(table);
        dataTables[tabName] = table;
    } else {
//        QScrollArea* scrollArea = new QScrollArea();
//        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
//        mainLayout->addWidget(scrollArea);
//        QVBoxLayout* subLayout = new QVBoxLayout(scrollArea);
//        scrollArea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//        mainLayout->setSizeConstraint(QLayout::SetFixedSize);
//        subLayout->setSizeConstraint(QLayout::SetMaximumSize);

//        for (int i = 0; i < 30; ++i) {
//            QLabel* testLabel = new QLabel;
//            testLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//            testLabel->setText("Sample text");
//            subLayout->addWidget(testLabel);
//        }
//        scrollArea->resize(300, 300);
    }

    tab->setLayout(mainLayout);
    tabs[tabName] = tab;

    return ui->displayerTab->addTab(tab, QString(tabName.c_str()));
}

void DataDisplayerWindow::setHeaders(std::string tabName, int columns)
{
    QTableWidget* table = dataTables[tabName];
    table->setColumnCount(columns);
}

void DataDisplayerWindow::appendData(std::string tabName, cv::Mat &rowData)
{
    QTableWidget* table = dataTables[tabName];
    table->setRowCount(table->rowCount() + 1);
    table->setColumnCount(rowData.cols + 1);

    // Insert time stamp
    table->setItem(table->rowCount() - 1, 0, new QTableWidgetItem(QString::number( QDateTime::currentDateTime().toMSecsSinceEpoch() )));

    for (int i = 1; i < rowData.cols + 1; ++i)
    {
        QTableWidgetItem* item;
        if (rowData.type() == CV_64F)
            item = new QTableWidgetItem(tr("%1").arg(rowData.at<double>(0, i - 1)));
        else
            item = new QTableWidgetItem(tr("%1").arg(rowData.at<float>(0, i - 1)));

        table->setItem(table->rowCount() - 1, i, item);
    }
}

int DataDisplayerWindow::addDisplay(std::string tabName, std::string displayName)
{
    QLabel* display = new QLabel;
    display->setText("Waiting for data");
    display->setTextFormat(Qt::RichText);
    QWidget* tab = tabs[tabName];
    QLayout* mainLayout = tab->layout();
    mainLayout->addWidget(display);
    tab->setLayout(mainLayout);

    displays[displayName] = display;
}

void DataDisplayerWindow::setDisplayValues(std::string tabName, std::string displayName, cv::Mat &displayData)
{
    cv::Point2f pointImage;
    pointImage.x = displayData.at<float>(0, 0);
    pointImage.y = displayData.at<float>(0, 1);
    cv::Point3f pointReal;
    pointReal.x = displayData.at<float>(0, 2);
    pointReal.y = displayData.at<float>(0, 3);
    pointReal.z = displayData.at<float>(0, 4);
    cv::Point3f pointHeightCorrection;
    pointHeightCorrection.x = displayData.at<float>(0, 5);
    pointHeightCorrection.y = displayData.at<float>(0, 6);
    pointHeightCorrection.z = displayData.at<float>(0, 7);
    cv::Point3f pointInterpolated;
    pointInterpolated.x = displayData.at<float>(0, 8);
    pointInterpolated.y = displayData.at<float>(0, 9);
    pointInterpolated.z = displayData.at<float>(0, 10);
    cv::Point3f pointInterpolatedCorrected;
    pointInterpolatedCorrected.x = displayData.at<float>(0, 11);
    pointInterpolatedCorrected.y = displayData.at<float>(0, 12);
    pointInterpolatedCorrected.z = displayData.at<float>(0, 13);

    QLabel* label = displays[displayName];
    //label->setText(tr("<html><b>%5</b><table><tr><td>Real World: </td><td style='font-size: 15px; font-weight: bold; color: #01a080;'>%1 </td><td style='font-size: 15px; font-weight: bold; color: #01a080;'> %2 </td></tr><tr><td>Image: </td><td style='font-weight: bold; color: #01a080;'>%3 </td><td style='font-weight: bold; color: #01a080;'> %4</td></tr></table><hr ></html>").arg(QString::number(pointReal.x, 'f', 2), QString::number(pointReal.y, 'f', 2), QString::number(pointImage.x, 'f', 2), QString::number(pointImage.y, 'f', 2), QString(displayName.c_str())));
    //label->setText(tr("<html><b>%5</b><table width='100%'><tr><td>Real World: </td><td style='font-size: 15px; font-weight: bold; color: #01a080;'>%1 </td><td style='font-size: 15px; font-weight: bold; color: #01a080;'> %2 </td><td>Image: </td><td style='font-weight: bold; color: #01a080;'>%3 </td><td style='font-weight: bold; color: #01a080;'> %4</td></tr></table><hr ></html>").arg(QString::number(pointReal.x, 'f', 2), QString::number(pointReal.y, 'f', 2), QString::number(pointImage.x, 'f', 2), QString::number(pointImage.y, 'f', 2), QString(displayName.c_str())));
    std::stringstream table;
    table << "<html>" << "<b>" << displayName << "</b> at image coordinates (" << QString::number(pointImage.x, 'f', 2).toStdString() << ", " << QString::number(pointImage.y, 'f', 2).toStdString() << ")";
    table << "<table width='100%'>";
    table << "<tr>" << "<td>H.Scaling</td>" << "<td>Raw</td>" << "<td>Interpolated</td>" << "<td>H.Corrected</td>" << "<td>H.Corr+Interp.</td>" << "</tr>";

    table << "<tr>";

    // Scaling with Homography
    table << "<td>No</td>";

    // Raw
    table << "<td>" << QString::number(pointReal.x, 'f', 2).toStdString() << ", " << QString::number(pointReal.y, 'f', 2).toStdString() << "</td>";
    // Interpolation
    table << "<td>" << QString::number(pointInterpolated.x, 'f', 2).toStdString() << ", " << QString::number(pointInterpolated.y, 'f', 2).toStdString() << "</td>";
    // Height Correction
    table << "<td>" << QString::number(pointHeightCorrection.x, 'f', 2).toStdString() << ", " << QString::number(pointHeightCorrection.y, 'f', 2).toStdString() << "</td>";
    // Height Correction plus Interpolation
    table << "<td>" << QString::number(pointInterpolatedCorrected.x, 'f', 2).toStdString() << ", " << QString::number(pointInterpolatedCorrected.y, 'f', 2).toStdString() << "</td>";

    table << "</tr>";

    if (displayData.cols == 28)
    {
        pointImage.x = displayData.at<float>(0, 14);
        pointImage.y = displayData.at<float>(0, 15);
        pointReal.x = displayData.at<float>(0, 16);
        pointReal.y = displayData.at<float>(0, 17);
        pointReal.z = displayData.at<float>(0, 18);
        pointHeightCorrection.x = displayData.at<float>(0, 19);
        pointHeightCorrection.y = displayData.at<float>(0, 20);
        pointHeightCorrection.z = displayData.at<float>(0, 21);
        pointInterpolated.x = displayData.at<float>(0, 22);
        pointInterpolated.y = displayData.at<float>(0, 23);
        pointInterpolated.z = displayData.at<float>(0, 24);
        pointInterpolatedCorrected.x = displayData.at<float>(0, 25);
        pointInterpolatedCorrected.y = displayData.at<float>(0, 26);
        pointInterpolatedCorrected.z = displayData.at<float>(0, 27);

        table << "<tr>";

        // Scaling with Homography
        table << "<td>Yes</td>";

        // Raw
        table << "<td>" << QString::number(pointReal.x, 'f', 2).toStdString() << ", " << QString::number(pointReal.y, 'f', 2).toStdString() << "</td>";
        // Interpolation
        table << "<td>" << QString::number(pointInterpolated.x, 'f', 2).toStdString() << ", " << QString::number(pointInterpolated.y, 'f', 2).toStdString() << "</td>";
        // Height Correction
        table << "<td>" << QString::number(pointHeightCorrection.x, 'f', 2).toStdString() << ", " << QString::number(pointHeightCorrection.y, 'f', 2).toStdString() << "</td>";
        // Height Correction plus Interpolation
        table << "<td>" << QString::number(pointInterpolatedCorrected.x, 'f', 2).toStdString() << ", " << QString::number(pointInterpolatedCorrected.y, 'f', 2).toStdString() << "</td>";

        table << "</tr>";
    }

    table << "</table>";
    table << "</html>";
    label->setText(tr(table.str().c_str()));
}

void DataDisplayerWindow::setPoseValues(std::string tabName, std::string displayName, cv::Mat &displayData)
{
    QLabel* label = displays[displayName];
    float roll = displayData.at<float>(0, 0);
    float pitch = displayData.at<float>(0, 1);
    float yaw = displayData.at<float>(0, 2);
    float positionX = displayData.at<float>(0, 3);
    float positionY = displayData.at<float>(0, 4);
    float positionZ = displayData.at<float>(0, 5);
    float targetRoll = displayData.at<float>(0, 6);
    float targetPitch = displayData.at<float>(0, 7);
    float targetYaw = displayData.at<float>(0, 8);
    float targetVerticalSpeed = displayData.at<float>(0, 9);
    float speedX = displayData.at<float>(0, 10);
    float speedY = displayData.at<float>(0, 11);

    std::string positiveStyle = "style=' background-color : green; color : white; '";
    std::string neutralStyle = "style=' background-color : white; color : black; '";
    std::string negativeStyle = "style=' background-color : red; color : white; '";
    std::stringstream table;
    table << "<html>" << "<b>" << displayName << "</b> ";
    table << "<table style='font-size:30px;' width='100%'>";
    table << "<tr>" << "<td width='300'>Roll</td>" << "<td align='right' " << (targetRoll > 0.5 ? positiveStyle : (targetRoll < 0.5) ? negativeStyle : neutralStyle )  << " >" << QString::number(targetRoll, 'f', 2).toStdString() << "</td></tr>";
    table << "<tr>" << "<td width='300'> - Roll Speed</td>" << "<td align='right' " << (speedX > 0.5 ? positiveStyle : (speedX < 0.5) ? negativeStyle : neutralStyle )  << " >" << QString::number(speedX, 'f', 2).toStdString() << "</td></tr>";
    table << "<tr>" << "<td width='300'>Pitch</td>" << "<td align='right' " << (targetPitch > 0.5 ? positiveStyle : (targetPitch < 0.5) ? negativeStyle : neutralStyle )  << ">" << QString::number(targetPitch, 'f', 2).toStdString() << "</td></tr>";
    table << "<tr>" << "<td width='300'> - Pitch Speed</td>" << "<td align='right' " << (speedY > 0.5 ? positiveStyle : (speedY < 0.5) ? negativeStyle : neutralStyle )  << " >" << QString::number(speedY, 'f', 2).toStdString() << "</td></tr>";
    table << "<tr>" << "<td width='300'>Yaw</td>" << "<td align='right' " << (targetYaw > 0.5 ? positiveStyle : (targetYaw < 0.5) ? negativeStyle : neutralStyle )  << ">" << QString::number(targetYaw, 'f', 2).toStdString() << "</td></tr>";
    table << "<tr>" << "<td width='300'>V.Speed</td>" << "<td align='right' " << (targetVerticalSpeed > 0.5 ? positiveStyle : (targetVerticalSpeed < 0.5) ? negativeStyle : neutralStyle )  << ">" << QString::number(targetVerticalSpeed, 'f', 2).toStdString()<< "</td></tr>";
    table << "</table>";
    table << "</html>";
    table << "</table>";
    table << "</html>";
    label->setText(tr(table.str().c_str()));
}

void DataDisplayerWindow::clearData(std::string tabName)
{
    QTableWidget* table = dataTables[tabName];
    table->setRowCount(0);
}

void DataDisplayerWindow::on_closeButton_clicked()
{
    this->close();
}

void DataDisplayerWindow::on_saveDataButton_clicked()
{
    QLayout* layout = ui->displayerTab->currentWidget()->layout();
    QWidget* target = layout->itemAt(0)->widget();
    QTableWidget* table = (QTableWidget*)target;
    if (table == qobject_cast<QTableWidget*>(target))
    {
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                        QString(),
                                                        tr("Comma Separated Files (*.csv)"));
        if (fileName != "")
        {
            QFile csvFile(fileName);
            if (csvFile.open(QFile::WriteOnly))
            {
                QTextStream out(&csvFile);
                for (int i = 0; i < table->rowCount(); ++i)
                {
                    for (int j = 0; j < table->columnCount(); ++j)
                    {
                        out << '"' << table->item(i, j)->text() << '"' << ",";
                    }
                    out << "\n";
                }
            }
            csvFile.close();
        }
    }
}

void DataDisplayerWindow::on_clearDataButton_clicked()
{
    QLayout* layout = ui->displayerTab->currentWidget()->layout();
    QWidget* target = layout->itemAt(0)->widget();
    QTableWidget* table = (QTableWidget*)target;
    if (table == qobject_cast<QTableWidget*>(target))
    {
        table->setRowCount(0);
    }
}

void DataDisplayerWindow::on_insertMarkButton_clicked()
{
    QLayout* layout = ui->displayerTab->currentWidget()->layout();
    QWidget* target = layout->itemAt(0)->widget();
    QTableWidget* table = (QTableWidget*)target;
    if (table == qobject_cast<QTableWidget*>(target))
    {
        table->setRowCount(table->rowCount() + 1);
        QDateTime date;

        for (int i = 0; i < table->columnCount(); ++i)
        {

            QTableWidgetItem* item;
            if (i == 0)
            {
                //qDebug() << "date.toMSecsSinceEpoch() " << date.currentDateTime().toMSecsSinceEpoch();
                qDebug() << "date.toTime_t() " << date.toTime_t() << "." <<  date.currentDateTime().toString("hh:mm:ss.zzz");
                item = new QTableWidgetItem(QString::number( date.currentDateTime().toMSecsSinceEpoch() ));
            }
            else if (i == 1)
            {
                item = new QTableWidgetItem(date.currentDateTime().toString());
            }
            else
            {
                item = new QTableWidgetItem(QString("999"));
            }
            table->setItem(table->rowCount() - 1, i, item);
        }
    }
}
