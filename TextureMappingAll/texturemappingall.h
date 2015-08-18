#ifndef TEXTUREMAPPINGALL_H
#define TEXTUREMAPPINGALL_H

#include <QtGui/QMainWindow>
#include "ui_texturemappingall.h"

class TextureMappingAll : public QMainWindow
{
	Q_OBJECT

public:
	TextureMappingAll(QWidget *parent = 0, Qt::WFlags flags = 0);
	~TextureMappingAll();

private:
	Ui::TextureMappingAllClass ui;
};

#endif // TEXTUREMAPPINGALL_H
