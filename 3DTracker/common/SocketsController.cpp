#include "SocketsController.h"
#include "common/TrackingCommon.h"

// 10.20.216.80 : 12345

SocketsController::SocketsController()
{
    receiving = false;
    qDebug() << "QUdpSocket created";
}

void SocketsController::initSocket(std::string address, int port)
{
    socket = new QUdpSocket(this);
    this->port = port;

    socket->bind(QHostAddress::Any, port);

    connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()) );
    connect(this, SIGNAL(setHapticInputSignal(int,int)), &TrackingCommon::parrotController, SLOT(setHapticInputSlot(int,int)) );
    connect(&TrackingCommon::parrotController, SIGNAL(writeParrotPositionToHapticSignal(std::string)), this, SLOT(writeDatagram(std::string)) );
    //connect(&TrackingCommon::mainController->, SIGNAL(writeParrotPositionToHapticSignal(std::string)), this, SLOT(writeDatagram(std::string)) );

    qDebug() << "QUdpSocket to " << address.c_str() << " inited";
}

void SocketsController::stopSocket()
{
    delete socket;
}

void SocketsController::readPendingDatagrams()
{
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        //qDebug() << "Datagram " << datagram << " from " << sender.toString();
        if (datagram.indexOf("A") == 0)
        {
            int bIndex = datagram.indexOf("B");
            int x = datagram.mid(1, bIndex - 1).toInt();
            int y = datagram.mid(bIndex + 1).toInt();
            qDebug() << "Datagram " << x << ", " << y <<" from " << sender.toString();

            // Initial point: -1000, 800 that is mapped to:
            // Final point: 1000, 800 that is mapped to:

            x += 1000;
            x *= 800.0 / 2000;
            if (x > 800)
                x = 800;
            else if (x < 0)
                x = 0;

            y += 1000;
            y *= 400.0 / 2000;
            if (y > 400)
                y = 400;
            else if (y < 0)
                y = 0;

            emit displayHapticInputSignal(x, y);
            emit setHapticInputSignal(x, y);
        }

        this->sender = sender;
        receiving = true;
    }
}

void SocketsController::writeDatagram(string data)
{
    if (receiving)
    {
        socket->writeDatagram(QByteArray(data.c_str()), sender, port);
    }
}
