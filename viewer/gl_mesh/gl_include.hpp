//
// Created by root on 2020/11/10.
//

#ifndef TSFUSION_GL_INCLUDE_HPP
#define TSFUSION_GL_INCLUDE_HPP

#ifdef _WIN32
#ifndef GLEW_STATIC
#define GLEW_STATIC
#endif
#endif

#include<GL/glew.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <QMainWindow>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QApplication>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QIcon>
#include <QTimer>

#include <ctime>
#include <iostream>
#include <fstream>


#endif //TSFUSION_GL_INCLUDE_HPP
