#ifndef SOCKETSCONTROLLER_H
#define SOCKETSCONTROLLER_H

#include <sstream>
#include <QUdpSocket>
#include <QHostAddress>

class SocketsController : public QUdpSocket {
    Q_OBJECT

signals:
    void setHapticInputSignal(int x, int y);
    void displayHapticInputSignal(int x, int y);

private:
    int port;
    bool input;
    std::string address;
    QUdpSocket *socket;
    bool receiving;
    QHostAddress sender;

private slots:
    void readPendingDatagrams();
    void writeDatagram(std::string data);

public:
    SocketsController();
    void initSocket(std::string address, int port);
    void stopSocket();

};

#endif // SOCKETSCONTROLLER_H
