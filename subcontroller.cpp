#include "subcontroller.h"
#include "stdio.h"
#include <QDebug>
#include <QMessageBox>

SubController::SubController()
{
    serial = new QSerialPort(this);
    instruction = undefined;
    status = idle;
    dataLength = 0;
    dataCount = 0;
    dataChecksum = 0;
    dataChecksumPacket = 0;
    connect(serial,SIGNAL(readyRead()),this,SLOT(readData()));
    connect(serial,SIGNAL(error(QSerialPort::SerialPortError)),this,SLOT(handleError(QSerialPort::SerialPortError)));
    for(int i=0; i<100; i++)
        buffer[i]=0;
    for(int i=0; i<20; i++){
        positionBuffer[i]=65535;
    }
}

bool SubController::begin(){
    serial->setPortName(setting.name);
    serial->setBaudRate(setting.baudRate);
    serial->setDataBits(setting.dataBits);
    serial->setParity(setting.parity);
    serial->setStopBits(setting.stopBits);
    serial->setFlowControl(setting.flowControl);
    qDebug()<<"connecting to "<<setting.name;
    if(serial->open(QIODevice::ReadWrite)){
        qDebug()<<"connect successfull";
        return true;
    }
    else {
        qDebug()<<"connect failed";
        return false;
    }
}

void SubController::end(){
    if(serial->isOpen())
        serial->close();
}

void SubController::setSettings(Settings s){
    setting = s;
}

bool SubController::isConnected(){
    return serial->isOpen();
}

void SubController::setTransmitMode(transmit_mode_t mode){
//    qDebug()<<"[subcontroller] setting transmit mode";
    uchar data[6];
    uchar length = 3;
    uchar cmd = transmit_mode;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd;
    data[4]=mode; checks += data[4];
    data[5]=~checks;
    serial->write((char*)data,6);
}

void SubController::setPositionReadMode(position_read_mode_t posRead){
//    qDebug()<<"[subcontroller] setting position read mode";
    uchar data[6];
    uchar length = 3;
    uchar cmd = position_read_mode;
    uchar checks = length + cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd;
    data[4]=posRead; checks += data[4];
    data[5]=~checks;
    serial->write((char*)data,6);
}

void SubController::update(char byte){
//    printf("[subcontroller] updating state, byte : 0x%x\n",byte&0xff);
//    qDebug()<<"[subcontroller] updating state, byte : "<<QString::number(byte,16);
    char rxdata = byte&0xff;
//    printf("[subcontroller] updating state, rxdata : 0x%x\n",rxdata);
    switch(status){
    case idle:{
        if((rxdata&0xff)==0xff)
            status=start_packet_1;
        break;
    }
    case start_packet_1:{
//        qDebug()<<"[subcontroller] starting packet 1, rxdata : "<<rxdata;
        if((rxdata&0xff)==0xff)
            status=start_packet_2;
        else
            status=idle;
        break;
    }
    case start_packet_2:{
        dataLength = rxdata&0xff;
//        qDebug()<<"[subcontroller] start packet 2, dataLenghth : "<<dataLength;
        dataChecksumPacket = dataLength;
        if(dataLength<255)
            status = length_rxd;
        else status = idle;
        break;
    }
    case length_rxd:{
        instruction = (instruction_t)rxdata;
        dataChecksumPacket += instruction;
        status = instruction_rxd;
        break;
    }
    case instruction_rxd:{
        dataCount = 0;
        buffer[dataCount++] = rxdata&0xff;
        dataChecksumPacket += rxdata&0xff;
        status = read_param;
        break;
    }
    case read_param:{
        if(dataCount>=dataLength-2){
            dataChecksum = rxdata&0xff;
            dataChecksumPacket = ~dataChecksumPacket;
            if(dataChecksum == dataChecksumPacket)
                commStatus = success;
            else
                commStatus = error;
            process();
            status = idle;
        }
        else {
            buffer[dataCount++] = rxdata&0xff;
            dataChecksumPacket += rxdata&0xff;
        }
        break;
    }
    default:
        status = idle;
        break;
    }
}

void SubController::process(){
    switch(instruction){
    case servo_instruction:{
        break;
    }
    case torque_instruction:{
        break;
    }
    case led_instruction:{
        break;
    }
    case position_request:{
        for(int i=0; i<20; i++){
            positionBuffer[i] = buffer[2*i]<<8 | buffer[2*i+1];
        }
//        qDebug()<<"[subcontroller] position buffer updated";
        emit positionUpdated();
        break;
    }
    case voltage_request:{
        for(int i=0; i<20; i++){
            voltageBuffer[i] = buffer[i];
        }
//        qDebug()<<"[subcontroller] voltage buffer updated";
        emit voltageUpdated();
        break;
    }
    case temperature_request:{
        for(int i=0; i<20; i++){
            temperatureBuffer[i] = buffer[i];
        }
//        qDebug()<<"[subcontroller] temperature buffer updated";
        emit temperatureUpdated();
        break;
    }
    case load_request:{
        uchar highByte, lowByte;
        for(int i=0; i<20; i++){
            highByte = (uchar)buffer[2*i]*256;
            lowByte = (uchar)buffer[2*i+1]&0xff;
            loadBuffer[i] = highByte+lowByte;
        }
//        qDebug()<<"[subcontroller] load buffer updated";
        emit loadUpdated();
        break;
    }
    case id_position_request:{
        int id = buffer[0]-1;
        uchar highByte = buffer[1]&0xff;
        uchar lowByte = buffer[2]&0xff;
//        quint16 position = (quint16)((highByte<<8)+lowByte);
        quint16 position = (highByte*256 + lowByte);
        if(position!=65535)
            positionBuffer[id] = position;
        emit positionUpdated();
        break;
    }
    default:
//        qDebug()<<"[subcontroller] unknown instruction";
        break;
    }
}

void SubController::sendPosition(int *pos){
    uchar data[45];
    uchar length = 42;
    uchar cmd = servo_instruction;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd;
    for(int i=0; i<20; i++){
        int idx = 2*i+4;
        data[idx] = pos[i]>>8;
        data[idx+1] = pos[i]&0xff;
        checks += data[idx]+data[idx+1];
    }
    data[44]=~checks;
    serial->write((char*)data,45);
}

void SubController::sendTorque(int *torque){
    uchar data[25];
    uchar length = 22;
    uchar cmd = torque_instruction;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd;
    for(int i=0; i<20; i++){
        data[i+4] = torque[i];
        checks += data[i+4];
    }
    data[24]=~checks;
    serial->write((char*)data,25);
}

void SubController::sendLED(int *led){
//    qDebug()<<"[subcontroller] sending led data";
    uchar data[25];
    uchar length = 22;
    uchar cmd = led_instruction;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd;
    for(int i=0; i<20; i++){
        data[i+4] = led[i];
        checks += data[i+4];
    }
    data[24]=~checks;
    if(serial->isOpen()){
//        qDebug()<<"sending data";
        serial->write((char*)data,25);
    }
}

void SubController::requestPosition(){
    uchar data[5];
    uchar length = 2;
    uchar cmd = position_request;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd; data[4]=~checks;
    serial->write((char*)data,5);
}

void SubController::requestVoltage(){
    uchar data[5];
    uchar length = 2;
    uchar cmd = voltage_request;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd; data[4]=~checks;
    serial->write((char*)data,5);
}

void SubController::requestTemperature(){
    uchar data[5];
    uchar length = 2;
    uchar cmd = temperature_request;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd; data[4]=~checks;
    serial->write((char*)data,5);
}

void SubController::requestLoad(){
    uchar data[5];
    uchar length = 2;
    uchar cmd = load_request;
    uchar checks = length+cmd;
    data[0]=0xff; data[1]=0xff; data[2]=length; data[3]=cmd; data[4]=~checks;
    serial->write((char*)data,5);
}

void SubController::readData(){
    //    qDebug()<<"buffer size "<<serial->readBufferSize();
    if(serial->bytesAvailable()>0){
        QByteArray data = serial->readAll();
//        qDebug()<<"[subcontroller] reading "<<data.count()<<" data : "<<data;
        for(int i=0; i<data.count(); i++){
            update(data.at(i));
        }
    }
}

void SubController::handleError(QSerialPort::SerialPortError e){
    qDebug()<<"serial error : "<<serial->errorString();
    if(error==QSerialPort::ResourceError){
        //        QMessageBox::critical(this,tr("Critical Error"),serial->errorString());
        //        end();
    }
}

void SubController::getLoad(int *load){
    for(int i=0; i<20; i++){
        load[i] = loadBuffer[i];
    }
}

void SubController::getPosition(int *position){
    for(int i=0; i<20; i++){
        position[i] = positionBuffer[i];
    }
}

void SubController::getVoltage(int *voltage){
    for(int i=0; i<20; i++){
        voltage[i] = voltageBuffer[i];
    }
}

void SubController::getTemperature(int *temperature){
    for(int i=0; i<20; i++){
        temperature[i] = temperatureBuffer[i];
    }
}
