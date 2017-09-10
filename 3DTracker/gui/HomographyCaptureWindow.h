#ifndef HOMOGRAPHYCAPTUREWINDOW_H
#define HOMOGRAPHYCAPTUREWINDOW_H

#include <QWidget>

namespace Ui {
class HomographyCaptureWindow;
}

class HomographyCaptureWindow : public QWidget
{
    Q_OBJECT
    
public:
    explicit HomographyCaptureWindow(QWidget *parent = 0);
    ~HomographyCaptureWindow();
    void addImagePoint(float x, float y);
    void findHomography(std::vector<float> imagePointsX, std::vector<float> imagePointsY, std::vector<float> realPointsX, std::vector<float> realPointsY);

private slots:
    void on_deleteHomographyEntry_clicked();

    void on_findHomographyButton_clicked();

private:
    Ui::HomographyCaptureWindow *ui;
};

#endif // HOMOGRAPHYCAPTUREWINDOW_H
