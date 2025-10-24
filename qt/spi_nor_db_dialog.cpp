/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "spi_nor_db_dialog.h"
#include "ui_spi_nor_db_dialog.h"

#define HEADER_LONG_WIDTH 120
#define HEADER_MED_WIDTH 110
#define HEADER_SHORT_WIDTH 50

SpiNorDbDialog::SpiNorDbDialog(SpiNorDb *chipDb, QWidget *parent) :
    QDialog(parent), ui(new Ui::SpiNorDbDialog),
    chipDbTableModel(chipDb, parent)
{
    ui->setupUi(this);

#ifdef Q_OS_WIN32
    QFont font("Courier New", 10);
    ui->chipDbTableView->setFont(font);
#endif

    chipDbProxyModel.setSourceModel(&chipDbTableModel);
    ui->chipDbTableView->setModel(&chipDbProxyModel);
    ui->chipDbTableView->setColumnWidth(SpiNorDb::CHIP_PARAM_NAME,
        HEADER_LONG_WIDTH);
    ui->chipDbTableView->setColumnWidth(SpiNorDb::CHIP_PARAM_PAGE_SIZE,
        HEADER_MED_WIDTH);
    ui->chipDbTableView->setColumnWidth(SpiNorDb::CHIP_PARAM_BLOCK_SIZE,
        HEADER_MED_WIDTH);
    ui->chipDbTableView->setColumnWidth(SpiNorDb::CHIP_PARAM_TOTAL_SIZE,
        HEADER_MED_WIDTH);

    for (int i = SpiNorDb::CHIP_PARAM_PAGE_OFF;
         i <= SpiNorDb::CHIP_PARAM_FREQ; i++)
    {
        ui->chipDbTableView->setColumnWidth(i, HEADER_MED_WIDTH);
    }

    connect(ui->addChipDbButton, SIGNAL(clicked()), this,
        SLOT(slotAddChipDbButtonClicked()));
    connect(ui->delChipDbButton, SIGNAL(clicked()), this,
        SLOT(slotDelChipDbButtonClicked()));
    connect(ui->okCancelButtonBox->button(QDialogButtonBox::Ok),
        SIGNAL(clicked()), this, SLOT(slotOkButtonClicked()));
    connect(ui->okCancelButtonBox->button(QDialogButtonBox::Cancel),
        SIGNAL(clicked()), this, SLOT(slotCancelButtonClicked()));
}

SpiNorDbDialog::~SpiNorDbDialog()
{
    delete ui;
}

void SpiNorDbDialog::slotAddChipDbButtonClicked()
{
    chipDbTableModel.addRow();
}

void SpiNorDbDialog::slotDelChipDbButtonClicked()
{
    QModelIndexList selection = ui->chipDbTableView->selectionModel()->
        selectedRows();

    if (!selection.count())
        return;

    chipDbTableModel.delRow(selection.at(0).row());
}

void SpiNorDbDialog::slotOkButtonClicked()
{
    chipDbTableModel.commit();
}

void SpiNorDbDialog::slotCancelButtonClicked()
{
    chipDbTableModel.reset();
}
