#include "camera.h"
#include <iostream>

#define TANGENT glm::normalize(glm::cross(look_, up_))

#define EPSILON 0.00001

namespace {
	float pan_speed = 0.1f;
	float roll_speed = 0.1f;
	float rotation_speed = 0.05f;
	float zoom_speed = 0.1f;
};

glm::mat4 Camera::get_view_matrix() const {
    glm::mat4 view_matrix = glm::mat4(1.0f);

    view_matrix[0] = glm::vec4(TANGENT, -glm::dot(eye_, TANGENT));
    view_matrix[1] = glm::vec4(up_, -glm::dot(eye_, up_));
    view_matrix[2] = glm::vec4(-look_, glm::dot(eye_, look_));
    view_matrix = glm::transpose(view_matrix);

    return view_matrix;
}

void Camera::toggle_fps() {
    fps = !fps;
}

void Camera::w() {
    if (accelerate_mode && zoom_speed*acceleration_const < terminal_velocity) {
        zoom_speed *= acceleration_const;
    }
    glm::vec3 old_center = glm::vec3(center_);
    if (!fps) {
        bool old_sign = (eye_ - center_).z < 0;
        bool new_sign = ((eye_ - look_*zoom_speed) - center_).z < 0;
        if ((old_sign ^ new_sign) == 1) { 
            can_zoom = false;
        }
    }
    eye_ += look_*zoom_speed;    
    if (fps) {
        center_ += look_*zoom_speed;
    }
}

void Camera::s() {
    if (accelerate_mode && zoom_speed*acceleration_const < terminal_velocity) {
        zoom_speed *= acceleration_const;
    }
    glm::vec3 old_center = glm::vec3(center_);
    bool old_sign = (eye_ - center_).z < 0;
    bool new_sign = ((eye_ + look_*zoom_speed) - center_).z < 0;
    if ((old_sign ^ new_sign) == 1) { 
        can_zoom = true;
    }
    eye_ -= look_*zoom_speed;    
    if (fps) {
        center_ -= look_*zoom_speed;
    }
}

void Camera::zoom_in() {
    acceleration_reset();
    if (!fps) {
        bool old_sign = (eye_ - center_).z < 0;
        bool new_sign = ((eye_ - look_*zoom_speed) - center_).z < 0;
        if ((old_sign ^ new_sign) == 1 || !can_zoom) {
            return;
        }
        w();
    }
}

void Camera::zoom_out() {
    acceleration_reset();
    if (!fps) {
        s();
    }
}

void Camera::a() {
    if (accelerate_mode && pan_speed*acceleration_const < terminal_velocity) {
        pan_speed *= acceleration_const;
    }
    eye_ -= TANGENT*pan_speed;    
}

void Camera::d() {
    if (accelerate_mode && pan_speed*acceleration_const < terminal_velocity) {
        pan_speed *= acceleration_const;
    }
    eye_ += TANGENT*pan_speed;    
}

void Camera::left() {
    if (accelerate_mode && roll_speed*acceleration_const < terminal_velocity) {
        roll_speed *= acceleration_const;
    }
    up_ = glm::rotate(up_, -roll_speed, look_);
}

void Camera::right() {
    if (accelerate_mode && roll_speed*acceleration_const < terminal_velocity) {
        roll_speed *= acceleration_const;
    }
    up_ = glm::rotate(up_, roll_speed, look_);
}

void Camera::up() {
    if (accelerate_mode && pan_speed*acceleration_const < terminal_velocity) {
        pan_speed *= acceleration_const;
    }
    eye_ += up_*pan_speed;
    center_ += up_*pan_speed;
}

void Camera::down() {
    if (accelerate_mode && pan_speed*acceleration_const < terminal_velocity) {
        pan_speed *= acceleration_const;
    }
    eye_ -= up_*pan_speed;
    center_ -= up_*pan_speed;
}

void Camera::rotate(glm::vec3 rotation_axis) {
    /* rotation_axis *= -1; */
    if (!fps) {
        eye_ = glm::rotate(eye_ - center_, -rotation_speed, rotation_axis);
        up_ = glm::rotate(up_, -rotation_speed, rotation_axis);
        look_ = glm::rotate(look_, -rotation_speed, rotation_axis);
        eye_ += center_;
    } else {
        center_ = glm::rotate(center_ - eye_, -rotation_speed, rotation_axis);
        up_ = glm::rotate(up_, -rotation_speed, rotation_axis);
        look_ = glm::rotate(look_, -rotation_speed, rotation_axis);
        center_ += eye_;
    }
}

void Camera::acceleration_toggle() {
    accelerate_mode = !accelerate_mode;
}

void Camera::acceleration_reset() {
	pan_speed = 0.1f;
	roll_speed = 0.1f;
	rotation_speed = 0.05f;
	zoom_speed = 0.1f;
}

glm::vec3 Camera::get_look() const {
    return look_;
}

bool Camera::isFPS() const {
    return fps;
}
