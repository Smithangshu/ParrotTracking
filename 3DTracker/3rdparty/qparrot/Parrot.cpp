
#include <QDebug>
#include "Parrot.h"

Parrot::Parrot()
{
    stop = false;
    move = false;
    calibracion = false;
}

void Parrot::connect()
{
    qDebug() << "Connecting to parrot";
    //establishing connection with the quadcopter
    heli = new CHeli();
    //initializing GUI and joystick
    //gui = new CGui(320,240);

    //this class holds the image from the drone
    //image = new CRawImage(320,240);
    //this class can segment the image
    //recognition = new CRecognition();
    //image->getSaveNumber();

    return;

    //nr=read(0, buf, sizeof(buf));

//    int i = 0;
//    while (stop == false){
//        //prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
//        //fprintf(stdout,"Angles %.2lf %.2lf %.2lf ",helidata.phi,helidata.psi,helidata.theta);
//        //fprintf(stdout,"Speeds %.2lf %.2lf %.2lf ",helidata.vx,helidata.vy,helidata.vz);
//        //fprintf(stdout,"Battery %.0lf ",helidata.battery);
//        //fprintf(stdout,"Largest blob %i %i\n",pos.x,pos.y);

//        //image is captured
//        heli->renewImage(image);

//        //finding a blob in the image
//        //pos = recognition->findSegment(image);

//        //turns the drone towards the colored blob
//        //yaw = 100*(pos.x-160); //uncomment to make the drone to turn towards a colored target

//        //getting input from the user
//        //if (nr=read(0, buf, sizeof(buf)))
//        //    processJoystick();
//        //processKeys();

//        //setting the drone angles
//        pitch=roll=yaw=height=0.0;
//        heli->setAngles(pitch,roll,yaw,height);

//        //drawing the image, the cross etc.
//        if (move==false || i%1 ==0){
//            //image->plotLine(pos.x,pos.y);
//            //image->plotCenter();
//            //gui->drawImage(image);
//            //gui->update();
//        }
//        i++;

//        usleep(20000);
//    }
}

void Parrot::disconnect()
{
    heli->land();
    delete heli;
}

void Parrot::takeOff()
{
    heli->takeoff();
}

void Parrot::land()
{
    heli->land();
}

void Parrot::setAngles(float roll, float pitch, float yaw, float verticalSpeed, int hover)
{
    heli->setAngles(-300.0 * pitch, 300.0 * roll, 300.0 * yaw, -300.0 * verticalSpeed, hover);
}

void Parrot::getHeliData(double &phi, double &psi, double &theta, double &altitude, double &battery, double &vx, double &vy, double &vz)
{
    heli->getHeliData(phi, psi, theta, altitude, battery, vx, vy, vz);
}



