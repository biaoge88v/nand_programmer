/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "spi_nor_info.h"

typedef struct __attribute__((__packed__))
{
    uint8_t page_offset;
    uint8_t read_cmd;
    uint8_t read_id_cmd;
    uint8_t write_cmd;
    uint8_t write_en_cmd;
    uint8_t erase_cmd;
    uint8_t status_cmd;
    uint8_t busy_bit;
    uint8_t busy_state;
    uint8_t addr_bytes;      // 新增：地址字节数 (3或4)
    uint8_t enter_4byte_cmd; // 新增：进入4字节模式命令
    uint8_t exit_4byte_cmd;  // 新增：退出4字节模式命令
    uint32_t freq;
} SpiNorConf;

SpiNorInfo::SpiNorInfo()
{
    hal = CHIP_HAL_SPI_NOR;
}

SpiNorInfo::~SpiNorInfo()
{
}

const QByteArray &SpiNorInfo::getHalConf()
{
    SpiNorConf conf;

    conf.page_offset = static_cast<uint8_t>(params[CHIP_PARAM_PAGE_OFF]);
    conf.read_cmd = static_cast<uint8_t>(params[CHIP_PARAM_READ_CMD]);
    conf.read_id_cmd = static_cast<uint8_t>(params[CHIP_PARAM_READ_ID_CMD]);
    conf.write_cmd = static_cast<uint8_t>(params[CHIP_PARAM_WRITE_CMD]);
    conf.write_en_cmd = static_cast<uint8_t>(params[CHIP_PARAM_WRITE_EN_CMD]);
    conf.erase_cmd = static_cast<uint8_t>(params[CHIP_PARAM_ERASE_CMD]);
    conf.status_cmd = static_cast<uint8_t>(params[CHIP_PARAM_STATUS_CMD]);
    conf.busy_bit = static_cast<uint8_t>(params[CHIP_PARAM_BUSY_BIT]);
    conf.busy_state = static_cast<uint8_t>(params[CHIP_PARAM_BUSY_STATE]);
    conf.addr_bytes = static_cast<uint8_t>(params[CHIP_PARAM_ADDR_BYTES]);      // 地址字节数参数
    conf.enter_4byte_cmd = static_cast<uint8_t>(params[CHIP_PARAM_ENTER_4BYTE]); // 进入4字节模式命令参数
    conf.exit_4byte_cmd = static_cast<uint8_t>(params[CHIP_PARAM_EXIT_4BYTE]);   // 退出4字节模式命令参数
    conf.freq = params[CHIP_PARAM_FREQ];

    halConf.clear();
    halConf.append(reinterpret_cast<const char *>(&conf), sizeof(conf));

    return halConf;
}

quint64 SpiNorInfo::getParam(uint32_t num)
{
    if (num >= CHIP_PARAM_NUM)
        return 0;

    return params[num];
}

int SpiNorInfo::setParam(uint32_t num, quint64 value)
{
    if (num >= CHIP_PARAM_NUM)
        return -1;

    params[num] = value;

    return 0;
}
