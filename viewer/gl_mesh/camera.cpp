//
// Created by root on 2020/8/27.
//
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif

#include<glm/gtx/transform.hpp>
#include "camera.h"

const float MOVE_SPEED = 0.01f;

void camera::mouseUpdate(const glm::vec2 &newMousePosition) {
    glm::vec2 mouseDelta = oldMousePosition - newMousePosition;
    viewDir = glm::mat3(glm::rotate(mouseDelta.x, UP)) * viewDir;
    oldMousePosition = newMousePosition;
}

void camera::panUpdate(const glm::vec2 &newMousePosition) {
    glm::vec2 panDelta = oldMousePosition - newMousePosition;
    if (panDelta.x > 0) {
        toRotateAround = glm::cross(viewDir, UP);
        position += MOVE_SPEED * toRotateAround;
    } else {
        toRotateAround = glm::cross(viewDir, UP);
        position += -MOVE_SPEED * toRotateAround;
    }
    oldMousePosition = newMousePosition;
}

void camera::rotateUpdate(const glm::vec2 &newMousePosition) {
    glm::vec2 mouseDelta = newMousePosition - oldMousePosition;
    if (glm::length(mouseDelta) > 50.0f) {
        oldMousePosition = newMousePosition;
        return;
    }
    const float ROTATIONAL_SPEED = 0.05f;
    toRotateAround = glm::cross(viewDir, UP);
    glm::mat4 rotator = glm::rotate(-mouseDelta.x * ROTATIONAL_SPEED, UP) *
                        glm::rotate(-mouseDelta.y * ROTATIONAL_SPEED, toRotateAround);
    viewDir = glm::mat3(rotator) * viewDir;

    oldMousePosition = newMousePosition;
}

void camera::moveForward() {
    position += viewDir * MOVE_SPEED;
}

void camera::moveBackward() {
    position += viewDir * (-MOVE_SPEED);

}

void camera::moveLeft() {
    toRotateAround = glm::cross(viewDir, UP);
    position += -MOVE_SPEED * toRotateAround;
}

void camera::moveRight() {
    toRotateAround = glm::cross(viewDir, UP);
    position += MOVE_SPEED * toRotateAround;
}

void camera::moveUp() {
    position += MOVE_SPEED * UP;
}

void camera::moveDown() {
    position += -MOVE_SPEED * UP;
}

glm::mat4 camera::getWorldtoViewMat() const {
    return glm::lookAt(position, position + viewDir, UP);
}