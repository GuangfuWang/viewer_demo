//
// Created by root on 2020/8/27.
//

#ifndef FUSION_MESHWINDOW_H
#define FUSION_MESHWINDOW_H

#include "gl_include.hpp"
#include <queue>
#include <mutex>
#include <string>
#include <condition_variable>
#include <QGLViewer/qglviewer.h>
#include "camera.h"


class meshWindow : public QGLViewer {
Q_OBJECT
public:
    meshWindow(QWidget *parent = 0,
               const char *name = 0, bool fs = false);

    ~meshWindow();

private:
    std::vector<float> verts;
    bool fullScreen;
    bool start_;
    bool colored;
    bool meshing_started;
    unsigned int color_array_size;
    unsigned int vert_array_size;
public:

    void genBuffer();
    void drawMesh();
    virtual void init();
    virtual void draw();

private:
    QTimer* t_;
    GLuint _vertBufferID;
    GLuint _colorBufferID;
    GLuint num_ver;
    GLuint programID;

private slots:
    void updateSlot();
    void changeColor();
};

#endif //FUSION_MESHWINDOW_H
