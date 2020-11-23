//
// Created by root on 2020/8/27.
//

#include"meshWindow.h"
#include <QGLViewer/manipulatedFrame.h>
#include <GL/gl.h>

using glm::mat4;

/**======================[de]constructors===========================*/

meshWindow::meshWindow(QWidget *parent, const char *name, bool fs) {
    fullScreen = fs;
    if (fullScreen) showFullScreen();
    this->t_ = new QTimer(this);
    t_->start(2000);
    start_= false;
    colored=true;
    meshing_started= false;
    color_array_size=3;
    vert_array_size=3;
    verts.resize(18);
    verts[0]=1.0f;
    verts[1]=0.0f;
    verts[2]=1.0f;
    verts[3]=0.5f;
    verts[4]=1.0f;
    verts[5]=0.0f;

    verts[6]=-1.0f;
    verts[7]=0.5f;
    verts[8]=1.0f;
    verts[9]=0.5f;
    verts[10]=0.0f;
    verts[11]=1.0f;

    verts[12]=1.0f;
    verts[13]=0.5f;
    verts[14]=5.0f;
    verts[15]=0.0f;
    verts[16]=1.0f;
    verts[17]=0.0f;

    connect(t_,SIGNAL(timeout()),this,SLOT(updateSlot()));
    printf("mesh window is now being generated,"
         "current update frequency is %d msec per update.",2000);
}

meshWindow::~meshWindow() {
    glUseProgram(0);
    glDeleteProgram(programID);
    delete t_;
}

/**=====================end of [de]constructors===================*/

#include<glm/glm.hpp>


void meshWindow::init() {
    setMouseTracking(true);
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, QGLViewer::CAMERA,
                    QGLViewer::ROTATE);
    setMouseBinding(Qt::NoModifier, Qt::RightButton, QGLViewer::CAMERA,
                    QGLViewer::TRANSLATE);
    setMouseBinding(Qt::NoModifier, Qt::MidButton, QGLViewer::CAMERA,
                    QGLViewer::ZOOM);
    setWheelBinding(Qt::NoModifier, QGLViewer::CAMERA, QGLViewer::ZOOM);

    setSnapshotFormat("PNG");
    setSnapshotCounter(0);

    setManipulatedFrame(new qglviewer::ManipulatedFrame());
    restoreStateFromFile();

    double m[3][3];
    m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f;
    m[1][0] = 0.0f; m[1][1] = -1.0f; m[1][2] = 0.0f;
    m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = -1.0f;
    qglviewer::Quaternion orientation;
    orientation.setFromRotationMatrix(m);
    camera()->setPosition(qglviewer::Vec(0.0f,0.0f,-2.0f));
    camera()->setOrientation(orientation);

    camera()->setFieldOfView(2.0f*atan(240.0/525.0));
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
}

void meshWindow::draw() {

    glMatrixMode(GL_PROJECTION);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    camera()->setZNearCoefficient (0.0001);
    camera()->setZClippingCoefficient (1000.0);

    glMultMatrixd(manipulatedFrame()->matrix());
    glScalef(1.0f, 1.0f, 1.0f);

    drawMesh();
    drawAxis(0.5);

    glPopMatrix();
}

void meshWindow::drawMesh() {
    if(vert_array_size>0){
        glColor3f(0.0f,1.0f,0.0f);
        if(!start_){
            genBuffer();
            start_ = true;
        }
        if(true){
            glBindBuffer(GL_ARRAY_BUFFER,_vertBufferID);
            glBufferData(GL_ARRAY_BUFFER,sizeof(float)*vert_array_size*6,verts.data(),GL_STATIC_DRAW);
            if(colored){
                glBindBuffer(GL_ARRAY_BUFFER,_colorBufferID);
                glBufferData(GL_ARRAY_BUFFER,sizeof(float)*vert_array_size*6,verts.data(),GL_STATIC_DRAW);
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER,_vertBufferID);
        glVertexPointer(3, GL_FLOAT, sizeof(float)*6, 0);
        if(color_array_size>0){
            glBindBuffer(GL_ARRAY_BUFFER,_colorBufferID);
            glColorPointer(3, GL_FLOAT, sizeof(float)*6, (char*)(sizeof(float) * 3));
        }
        else{
            glColor3f(0.5f,0.5f,0.5f);
        }

        glEnableClientState(GL_VERTEX_ARRAY);
        if(colored){
            glEnableClientState(GL_COLOR_ARRAY);
        }else{
            glColor3f(0.5f,0.5f,0.5f);
        }

        glPolygonMode(GL_FRONT, GL_FILL);
        glPolygonMode(GL_BACK, GL_FILL);

        glDrawArrays(GL_TRIANGLES,0,vert_array_size);

        if(colored) glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

}

void meshWindow::updateSlot() {

    update();
}

void meshWindow::genBuffer() {
    glewInit();
    glGenBuffers(1, &_vertBufferID);
    glGenBuffers(1, &_colorBufferID);
}

void meshWindow::changeColor() {
    colored=!colored;
}








