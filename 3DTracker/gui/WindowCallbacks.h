#ifndef WINDOWCALLBACKS_H
#define WINDOWCALLBACKS_H

#include "opencv2/core/core.hpp"

class WindowCallBack
{
public:
    void virtual mousePress(int x, int y);
    void virtual mouseMove(int x, int y);
    void virtual mouseRelease(int x, int y);
};

class TrackingCallback : public WindowCallBack
{
public:
    void mousePress(int x, int y);
    void mouseMove(int x, int y);
    void mouseRelease(int x, int y);
};

void trackingCallbackProcessing(int event, int x, int y, int flags, void* param, int box_id);
void homographyCallback( int event, int x, int y, int flags, void* param);

#endif // WINDOWCALLBACKS_H
