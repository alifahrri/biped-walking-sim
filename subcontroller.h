#ifndef SUBCONTROLLER_H
#define SUBCONTROLLER_H

#include <QSerialPort>
#include <QtCore>

class SubController : public QObject
{
    Q_OBJECT
public:
    struct Settings {
        QString name;
        qint32 baudRate;
        QString stringBaudRate;
        QSerialPort::DataBits dataBits;
        QString stringDataBits;
        QSerialPort::Parity parity;
        QString stringParity;
        QSerialPort::StopBits stopBits;
        QString stringStopBits;
        QSerialPort::FlowControl flowControl;
        QString stringFlowControl;
        bool localEchoEnabled;
    };

    enum status_t {idle, start_packet_1, start_packet_2, length_rxd, instruction_rxd, read_param, checksum} status;
    enum instruction_t {servo_instruction = 0x01, torque_instruction = 0x02, led_instruction = 0x03
                        , position_request = 0x10, voltage_request = 0x20
                        , temperature_request = 0x30, load_request = 0x40
                        , id_position_request = 0x50
                        , transmit_mode = 0xF0
                        , position_read_mode = 0xF1
                        , undefined = 0x00} instruction;
    enum communication_status_t {error = 0, success = 1} commStatus;
    enum transmit_mode_t {no_operation = 0x00, position_mode = 0x01, position_torque_mode = 0x02} transmitMode;
    enum position_read_mode_t {position_one_shot = 0x00, position_stream = 0x01};

    bool isConnected();
    bool begin();
    void end();
    void setSettings(Settings s);
    void setTransmitMode(transmit_mode_t mode);
    void setPositionReadMode(position_read_mode_t posRead);
    void update(char byte);
    void sendPosition(int* pos);
    void sendTorque(int* torque);
    void sendLED(int* led);
    void requestPosition();
    void requestVoltage();
    void requestTemperature();
    void requestLoad();
    void getPosition(int* position);
    void getVoltage(int* voltage);
    void getTemperature(int* temperature);
    void getLoad(int* load);

private slots:
    void readData();
    void handleError(QSerialPort::SerialPortError e);

private:
    void process();

private:
    int positionBuffer[20];
    int voltageBuffer[20];
    int temperatureBuffer[20];
    int loadBuffer[20];
    int dataLength;
    int dataCount;
    char dataChecksum;
    char dataChecksumPacket;
    char buffer[100];

public:
    SubController();

private:
    Settings setting;
    QSerialPort *serial;

signals:
    void positionUpdated();
    void voltageUpdated();
    void temperatureUpdated();
    void loadUpdated();
};

#endif // SUBCONTROLLER_H
