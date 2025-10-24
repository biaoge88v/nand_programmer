/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef PROGRAMMER_H
#define PROGRAMMER_H

#include <QObject>
#include <QByteArray>
#include <cstdint>
#include "writer.h"
#include "reader.h"
#include "cmd.h"
#include "parallel_chip_db.h"
#include "spi_nor_db.h"
#include "serial_port.h"

using namespace std;

class Programmer : public QObject
{
    Q_OBJECT

    typedef enum
    {
        FIRMWARE_IMAGE_1    = 0,
        FIRMWARE_IMAGE_2    = 1,
        FIRMWARE_IMAGE_LAST = 2,
    } FirmwareImageNum;

    typedef struct
    {
        FirmwareImageNum num;
        uint32_t address;
        uint32_t offset;
        uint32_t size;
    } FirmwareImage;

    const FirmwareImage firmwareImage[FIRMWARE_IMAGE_LAST] =
    {
        { FIRMWARE_IMAGE_1, 0x08004000, 0x00004000, 0x1e000 },
        { FIRMWARE_IMAGE_2, 0x08022000, 0x00022000, 0x1e000 },
    };
    const uint32_t flashPageSize = 0x100;

    SerialPort serialPort;
    QString usbDevName;
    Writer writer;
    Reader reader;
    bool isConn;
    bool skipBB;
    bool incSpare;
    bool enableHwEcc;
    FwVersion fwVersion;
    uint8_t activeImage;
    uint8_t updateImage;
    QString firmwareFileName;
    QFile FirmwareFile;
    QVector<uint8_t> buf;
    ChipId *chipId_p;

    int firmwareImageRead();
    void firmwareUpdateStart();

public:
    QByteArray writeData;

    explicit Programmer(QObject *parent = nullptr);
    ~Programmer();
    int connect();
    void disconnect();
    bool isConnected();
    void setUsbDevName(const QString &name);
    QString getUsbDevName();
    bool isSkipBB();
    void setSkipBB(bool skip);
    bool isIncSpare();
    void setIncSpare(bool incSpare);
    bool isHwEccEnabled();
    void setHwEccEnabled(bool isHwEccEnabled);
    void readChipId(ChipId *chipId);
    void eraseChip(quint64 addr, quint64 len);
    void readChip(QVector<uint8_t> *buf, quint64 addr, quint64 len, bool isReadLess);
    void writeChip(QVector<uint8_t> *buf, quint64 addr, quint64 len,
        uint32_t pageSize);
    void readChipBadBlocks();
    void confChip(ChipInfo *chipInfo);
    void detectChip();
    QString fwVersionToString(FwVersion fwVersion);
    void firmwareUpdate(const QString &fileName);

signals:
    void connectCompleted(quint64 ret);
    void readChipIdCompleted(quint64 ret);
    void writeChipCompleted(int ret);
    void writeChipProgress(quint64 progress);
    void readChipCompleted(quint64 ret);
    void readChipProgress(quint64 ret);
    void eraseChipCompleted(quint64 ret);
    void eraseChipProgress(quint64 progress);
    void readChipBadBlocksProgress(quint64 progress);
    void readChipBadBlocksCompleted(quint64 ret);
    void confChipCompleted(quint64 ret);
    void firmwareUpdateCompleted(int ret);
    void firmwareUpdateProgress(quint64 progress);

private slots:
    void readChipIdCb(quint64 ret);
    void writeCb(int ret);
    void writeProgressCb(quint64 progress);
    void readCb(quint64 ret);
    void readProgressCb(quint64 progress);
    void eraseChipCb(quint64 ret);
    void eraseProgressChipCb(quint64 progress);
    void readChipBadBlocksCb(quint64 ret);
    void readChipBadBlocksProgressCb(quint64 progress);
    void confChipCb(quint64 ret);
    void logCb(QtMsgType msgType, QString msg);
    void connectCb(quint64 ret);
    void disconnected();
    void getActiveImageCb(quint64 ret);
    void firmwareUpdateCb(int ret);
    void firmwareUpdateProgressCb(quint64 progress);
};

#endif // PROGRAMMER_H
