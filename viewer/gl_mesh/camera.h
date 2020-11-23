//
// Created by root on 2020/8/27.
//

#ifndef FUSION_CAMERA_H
#define FUSION_CAMERA_H

#include<glm/glm.hpp>

using glm::vec3;

class camera {
private:
    glm::vec3 position;
    glm::vec3 viewDir;
    glm::vec3 UP;
    glm::vec3 toRotateAround;
    glm::vec2 oldMousePosition;

public:
    camera() : viewDir(0.0f, 0.0f, -1.0f), UP(0.0f, 1.0f, 0.0f), position(0.0f, 0.0f, 0.0f) {};

    void mouseUpdate(const glm::vec2 &newMousePosition);

    void panUpdate(const glm::vec2 &newMousePosition);

    void rotateUpdate(const glm::vec2 &newMousePosition);

    void moveForward();

    void moveBackward();

    void moveLeft();

    void moveRight();

    void moveUp();

    void moveDown();

    glm::mat4 getWorldtoViewMat() const;

};

#endif //FUSION_CAMERA_H
