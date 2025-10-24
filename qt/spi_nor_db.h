/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef SPI_NOR_DB_H
#define SPI_NOR_DB_H

#include "chip_db.h"
#include "spi_nor_info.h"

#include <cstdint>
#include <QString>
#include <QObject>
#include <QVector>
#include <QFile>

class SpiNorDb : public ChipDb
{
private:
    QString dbFileName = "nando_spi_nor_db.csv";

protected:
    QString getDbFileName() override;
    ChipInfo *stringToChipInfo(const QString &s) override;
    int chipInfoToString(ChipInfo *chipInfo, QString &s) override;

public:
    enum
    {
        CHIP_PARAM_NAME,
        CHIP_PARAM_PAGE_SIZE,
        CHIP_PARAM_BLOCK_SIZE,
        CHIP_PARAM_TOTAL_SIZE,
        CHIP_PARAM_PAGE_OFF,
        CHIP_PARAM_READ_CMD,
        CHIP_PARAM_READ_ID_CMD,
        CHIP_PARAM_WRITE_CMD,
        CHIP_PARAM_WRITE_EN_CMD,
        CHIP_PARAM_ERASE_CMD,
        CHIP_PARAM_STATUS_CMD,
        CHIP_PARAM_BUSY_BIT,
        CHIP_PARAM_BUSY_STATE,
        CHIP_PARAM_FREQ,
        CHIP_PARAM_ID1,
        CHIP_PARAM_ID2,
        CHIP_PARAM_ID3,
        CHIP_PARAM_ID4,
        CHIP_PARAM_ID5,
        CHIP_PARAM_NUM
    };

    explicit SpiNorDb();
    virtual ~SpiNorDb();

    ChipInfo *chipInfoGetByName(QString name);
    int getIdByChipId(uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4,
        uint32_t id5);
    QString getNameByChipId(uint32_t id1, uint32_t id2,
        uint32_t id3, uint32_t id4, uint32_t id5, uint32_t id6) override;
    quint64 getChipParam(int chipIndex, int paramIndex);
    int setChipParam(int chipIndex, int paramIndex, quint64 paramValue);
};

#endif // SPI_NOR_DB_H
