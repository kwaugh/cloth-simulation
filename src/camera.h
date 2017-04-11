#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include "glm/gtc/matrix_transform.hpp"

class Camera {
public:
	glm::mat4 get_view_matrix() const;
	// FIXME: add functions to manipulate camera objects.
    void toggle_fps();
    void w();
    void s();
    void zoom_in();
    void zoom_out();
    void a();
    void d();
    void left();
    void right();
    void up();
    void down();
    void rotate(glm::vec3 rotation_axis);
    void acceleration_toggle();
    void acceleration_reset();

    glm::vec3 get_look() const;
    bool isFPS() const;
private:
	float camera_distance_ = 3.0;
	glm::vec3 look_ = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 up_ = glm::vec3(0.0f, 1.0, 0.0f);
	glm::vec3 eye_ = glm::vec3(0.0f, 0.0f, camera_distance_);
    glm::vec3 center_ = glm::vec3(0.0f, 0.0f, 0.0f);
    bool fps = false;
    bool can_zoom = true;
    bool accelerate_mode = false;
    static constexpr float acceleration_const = 1.03f;
    static constexpr float terminal_velocity = 4.0f;
};

#endif
