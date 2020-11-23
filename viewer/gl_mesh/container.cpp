//
// Created by root on 2020/8/28.
//
#include "container.h"

#include <unistd.h>

std::string getCurrentDir() {
    char vertexPath[75];
    getcwd(vertexPath, 75);
    std::string dir=get_current_dir_name();
    if(dir.find("bin")!=std::string::npos)
        dir = dir.replace(dir.find_last_of('/') + dir.begin(), dir.end(), "");
    return dir;
}

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent) {
    std::string path = getCurrentDir();

    path += "/viewer/resources/windowicon.png";
    QIcon icon = *new QIcon(path.c_str());
    setWindowIcon(icon);
    setGeometry(320, 240, 640, 480);
    setMinimumHeight(480);
    setMinimumWidth(640);
    setWindowTitle("thu_fusion");

    glWidget = new meshWindow();

    this->setCentralWidget(glWidget);
    QAction* switch_color=new QAction();
    this->addAction(switch_color);
    switch_color->setShortcut(Qt::Key::Key_Space);
    connect(switch_color,SIGNAL(triggered()),glWidget,SLOT(changeColor()));

    //mesh window will handle key press event.
    glWidget->setFocusPolicy(Qt::StrongFocus);
    createActions();
    createMenus();
}

void MainWindow::ShortCutInfo() {
    QMessageBox::about(this, tr("Shortcut Info"), tr("<h3>Shortcuts:</h3><br><b>Save mesh to .ply file:</b>"));
}

void MainWindow::about() {
    QMessageBox::about(this, tr("About fusion"),
                       tr("<h3>About this project:</h3><br>"
                          "<b>Dependencies:</b> Eigen-3.3 or above, OpenCV-3.3.7, QGLViewer-2.7.2, Qt-5.9.5, intel MKL,etc.<br>"
                          "<b>Version:</b> 0.0.1.<br>"
                          "<b>Author:</b> Guangfu<br>"
                          "<b>From:</b> Mechanical Department, Tsinghua University, China.<br>"
                          "<br>for more information please visit: <a href=\"https://www.baidu.com/\">https://www.baidu.com</a>."));
}

void MainWindow::createActions() {
    ply = new QAction(tr("&Save as .ply"), this);
    ply->setShortcut(tr("Ctrl+P"));
    ply->setToolTip(tr("yes, trigger it"));
    connect(ply, SIGNAL(triggered()),
            this, SLOT(about()));

    stl = new QAction(tr("&Save as .stl"), this);
    stl->setShortcut(tr("Ctrl+S"));
    connect(stl, SIGNAL(triggered()),
            this, SLOT(about()));

    obj = new QAction(tr("&Save as .obj"), this);
    obj->setShortcut(tr("Ctrl+O"));
    connect(obj, SIGNAL(triggered()), this, SLOT(about()));

    pose = new QAction(tr("&Export trajectories (KITTI format)"), this);
    pose->setShortcut(tr("Ctrl+Alt+P"));
    connect(pose, SIGNAL(triggered()), this, SLOT(about()));

    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcuts(QKeySequence::Quit);
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

    parallelSetting = new QAction(tr("&Parallel Settings"), this);
    connect(parallelSetting, SIGNAL(triggered()), this, SLOT(about()));

    dispalyMode = new QAction(tr("&Display Mode"), this);
    connect(dispalyMode, SIGNAL(triggered()), glWidget, SLOT(changeColor()));

    log = new QAction(tr("&Log output"), this);
    connect(log, SIGNAL(triggered()), this, SLOT(about()));

    initPoint = new QAction(tr("&Init Point"), this);
    connect(initPoint, SIGNAL(triggered()), this, SLOT(about()));

    showGrid = new QAction(tr("&Show Grid"), this);
    connect(showGrid, SIGNAL(triggered()), this, SLOT(about()));

    memoryInfo = new QAction(tr("&Show Memory Info"), this);
    connect(memoryInfo, SIGNAL(triggered()), this, SLOT(about()));

    timeInfo = new QAction(tr("&Show Frequency"), this);
    connect(timeInfo, SIGNAL(triggered()), this, SLOT(about()));

    computingInfo = new QAction(tr("&Show Computing Info"), this);
    connect(computingInfo, SIGNAL(triggered()), this, SLOT(about()));

    shortcut = new QAction(tr("&Shortcuts"), this);
    shortcut->setShortcut(tr("Ctrl+Alt+S"));
    connect(shortcut, SIGNAL(triggered()), this, SLOT(ShortCutInfo()));

    aboutAction = new QAction(tr("&About"), this);
    connect(aboutAction, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAction = new QAction(tr("About &Qt"), this);
    connect(aboutQtAction, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
}

void MainWindow::createMenus() {
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(ply);
    fileMenu->addAction(stl);
    fileMenu->addAction(obj);
    fileMenu->addSeparator();
    fileMenu->addAction(pose);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);

    settingMenu = menuBar()->addMenu(tr("&Settings"));

    settingMenu->addAction(parallelSetting);
    settingMenu->addAction(dispalyMode);
    settingMenu->addAction(log);
    settingMenu->addAction(initPoint);
    settingMenu->addAction(showGrid);

    statisticMenu = menuBar()->addMenu(tr("&Statistics"));

    statisticMenu->addAction(memoryInfo);
    statisticMenu->addAction(timeInfo);
    statisticMenu->addAction(computingInfo);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(shortcut);
    helpMenu->addSeparator();
    helpMenu->addAction(aboutAction);
    helpMenu->addAction(aboutQtAction);
}

MainWindow::~MainWindow() {
}


void MainWindow::update() {

}

void MainWindow::savePLY() {

}

void MainWindow::saveSTL() {

}

void MainWindow::saveOBJ() {

}

void MainWindow::parallel_setting() {

}

void MainWindow::display_mode_switch() {

}

void MainWindow::show_memory() {

}

void MainWindow::show_frequency() {

}

void MainWindow::show_grid() {

}

void MainWindow::show_computing_info() {

}

void MainWindow::export_pose() {

}




