//
// Created by root on 2020/8/28.
//
#ifndef FUSION_CONTAINER_H
#define FUSION_CONTAINER_H

#include "gl_include.hpp"
#include "meshWindow.h"
#include <QGLViewer/qglviewer.h>


class MainWindow : public QMainWindow {
Q_OBJECT


public:
    MainWindow(QWidget *parent = 0);

    ~MainWindow();

    void update();


private slots:

    void about();

    void ShortCutInfo();

    void savePLY();

    void saveSTL();

    void saveOBJ();

    void parallel_setting();

    void display_mode_switch();

    void show_memory();

    void show_frequency();

    void show_grid();

    void show_computing_info();

    void export_pose();

private:
    void createMenus();

    void createActions();

    QVBoxLayout *layout;
    meshWindow *glWidget;

    QMenu *fileMenu;
    QMenu *settingMenu;
    QMenu *statisticMenu;
    QMenu *helpMenu;
    QAction *ply;
    QAction *stl;
    QAction *obj;
    QAction *pose;
    QAction *parallelSetting;
    QAction *dispalyMode;
    QAction *log;
    QAction *initPoint;
    QAction *showGrid;
    QAction *memoryInfo;
    QAction *timeInfo;
    QAction *computingInfo;
    QAction *exitAction;
    QAction *shortcut;
    QAction *aboutAction;
    QAction *aboutQtAction;
};

#endif //FUSION_CONTAINER_H
