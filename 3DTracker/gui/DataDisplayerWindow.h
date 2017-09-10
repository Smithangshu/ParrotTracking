#ifndef DATADISPLAYERWINDOW_H
#define DATADISPLAYERWINDOW_H

#include <QWidget>
#include <map>
#include <QTableWidget>
#include <QLabel>
#include <QTabWidget>
#include "opencv2/core/core.hpp"

namespace Ui {
class DataDisplayerWindow;
}

class DataDisplayerWindow : public QWidget
{
    Q_OBJECT
    
public:
    explicit DataDisplayerWindow(QWidget *parent = 0);
    ~DataDisplayerWindow();
    int addTab(std::string tabName, bool insertTable = true);
    void setHeaders(std::string tabName, int columns);
    void setDisplayValues(std::string tabName, std::string displayName, cv::Mat &displayData);
    void setPoseValues(std::string tabName, std::string displayName, cv::Mat &displayData);
    int addDisplay(std::string tabName, std::string displayName);
    void appendData(std::string tabName, cv::Mat &rowData);
    void clearData(std::string tabName);

private slots:
    void on_closeButton_clicked();

    void on_saveDataButton_clicked();

    void on_clearDataButton_clicked();

    void on_insertMarkButton_clicked();

private:
    Ui::DataDisplayerWindow *ui;
    std::map<std::string, QWidget*> tabs;
    std::map<std::string, QTableWidget*> dataTables;
    std::map<std::string, QLabel*> displays;
};

#endif // DATADISPLAYERWINDOW_H
