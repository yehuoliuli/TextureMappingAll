/********************************************************************************
** Form generated from reading UI file 'texturemappingall.ui'
**
** Created: Thu Aug 13 16:50:22 2015
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TEXTUREMAPPINGALL_H
#define UI_TEXTUREMAPPINGALL_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TextureMappingAllClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *TextureMappingAllClass)
    {
        if (TextureMappingAllClass->objectName().isEmpty())
            TextureMappingAllClass->setObjectName(QString::fromUtf8("TextureMappingAllClass"));
        TextureMappingAllClass->resize(600, 400);
        menuBar = new QMenuBar(TextureMappingAllClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        TextureMappingAllClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(TextureMappingAllClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        TextureMappingAllClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(TextureMappingAllClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        TextureMappingAllClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(TextureMappingAllClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        TextureMappingAllClass->setStatusBar(statusBar);

        retranslateUi(TextureMappingAllClass);

        QMetaObject::connectSlotsByName(TextureMappingAllClass);
    } // setupUi

    void retranslateUi(QMainWindow *TextureMappingAllClass)
    {
        TextureMappingAllClass->setWindowTitle(QApplication::translate("TextureMappingAllClass", "TextureMappingAll", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TextureMappingAllClass: public Ui_TextureMappingAllClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TEXTUREMAPPINGALL_H
