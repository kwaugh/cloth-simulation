#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

// OpenGL library includes
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <debuggl.h>
#include <atomic>
#include "camera.h"
#include "simulation.h"

#define RENDER_STEPS 20

using namespace std;

int window_width = 800, window_height = 600;

// VBO and VAO descriptors.
enum { kVertexBuffer, kIndexBuffer, kNumVbos };

// These are our VAOs.
enum { kGeometryVao, kFloorVao, kNumVaos };

GLuint g_array_objects[kNumVaos];  // This will store the VAO descriptors.
GLuint g_buffer_objects[kNumVaos][kNumVbos];  // These will store VBO descriptors.

vector<glm::vec4> obj_vertices;
vector<glm::uvec3> obj_faces;

atomic_bool shouldRender(true);

/*
// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;

void main(){
// Output position of the vertex, in clip space : MVP * position
gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
}
*/
// C++ 11 String Literal
// See http://en.cppreference.com/w/cpp/language/string_literal
const char* vertex_shader =
#include "shaders/default.vert"
;

const char* geometry_shader =
#include "shaders/default.geom"
;

const char* fragment_shader =
#include "shaders/default.frag"
;

const char* floor_fragment_shader =
#include "shaders/floor.frag"
;

void takeSteps(shared_ptr<Simulation> sim) {
    while (true) {
        for (int i = 0; i < RENDER_STEPS; i++) {
            /* cout << "i: " << i << endl; */
            sim->takeSimulationStep();
        }
        shouldRender.store(true);
    }
}

void CreateTriangle(vector<glm::vec4>& vertices,
        vector<glm::uvec3>& indices) {
    uint sz = vertices.size();
    // vertices.push_back(glm::vec4(-0.5f, 0.5f, -2.5f, 1.0f));
    // vertices.push_back(glm::vec4(0.5f, -0.5f, -2.5f, 1.0f));
    // vertices.push_back(glm::vec4(0.5f, 0.5f, -2.5f, 1.0f));
    double zOffset = -0.0;
    vertices.push_back(glm::vec4(-0.5f, -0.5f, -0.5f + zOffset, 1.0f));
    vertices.push_back(glm::vec4(0.5f, -0.5f, -0.5f + zOffset, 1.0f));
    vertices.push_back(glm::vec4(0.0f, 0.5f, -0.5f + zOffset, 1.0f));
    indices.push_back(sz+glm::uvec3(0, 1, 2));
}

void SaveObj() {
    ofstream ofs;
    ofs.open ("geometry.obj", ofstream::out | ofstream::trunc);
    for (uint i = 0; i < obj_vertices.size(); i++) {
        ofs << "v " << obj_vertices[i].x << " " << obj_vertices[i].y << " " <<
            obj_vertices[i].z << "\n";
    }
    for (uint i = 0; i < obj_faces.size(); i++) {
        glm::uvec3 adjusted_face = glm::uvec3(obj_faces[i].x + 1,
                obj_faces[i].y + 1, obj_faces[i].z + 1);;
        ofs << "f " << adjusted_face.x << " " << adjusted_face.y << " " <<
            adjusted_face.z << "\n";
    }
    ofs.close();
}

void ErrorCallback(int error, const char* description) {
    cerr << "GLFW Error " << error << ": " << description << "\n";
}

shared_ptr<Simulation> sim;
Camera g_camera;


void KeyCallback(GLFWwindow* window,
        int key,
        int scancode,
        int action,
        int mods) {
    // Note:
    // This is only a list of functions to implement.
    // you may want to re-organize this piece of code.
    if (action == GLFW_RELEASE) {
        g_camera.acceleration_reset();
    }

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    else if (key == GLFW_KEY_S && mods == GLFW_MOD_CONTROL && action ==
            GLFW_RELEASE) {
        SaveObj();
    } else if (key == GLFW_KEY_W && action != GLFW_RELEASE) {
        g_camera.w();
    } else if (key == GLFW_KEY_S && action != GLFW_RELEASE && mods !=
            GLFW_MOD_CONTROL) {
        g_camera.s();
    } else if (key == GLFW_KEY_A && action != GLFW_RELEASE) {
        g_camera.a();
    } else if (key == GLFW_KEY_D && action != GLFW_RELEASE) {
        g_camera.d();
    } else if (key == GLFW_KEY_LEFT && action != GLFW_RELEASE) {
        g_camera.left();
    } else if (key == GLFW_KEY_RIGHT && action != GLFW_RELEASE) {
        g_camera.right();
    } else if (key == GLFW_KEY_DOWN && action != GLFW_RELEASE) {
        g_camera.down();
    } else if (key == GLFW_KEY_UP && action != GLFW_RELEASE) {
        g_camera.up();
    } else if (key == GLFW_KEY_C && action != GLFW_RELEASE) {
        g_camera.toggle_fps();
    } else if (key == GLFW_KEY_L && action == GLFW_RELEASE) {
        g_camera.acceleration_toggle();
    }
}

int g_current_button;
bool g_mouse_pressed;
glm::dvec2 g_mouse_pos;
glm::mat4 projection_matrix;
glm::mat4 inverse_projection_matrix;

void MousePosCallback(GLFWwindow* window, double mouse_x, double mouse_y) {
    if (!g_mouse_pressed)
        return;
    if (g_current_button == GLFW_MOUSE_BUTTON_LEFT) {
        glm::vec4 move_vec = glm::vec4(g_mouse_pos.x - mouse_x, mouse_y - g_mouse_pos.y, 0, 0);
        glm::vec3 global_move_vec = glm::vec3(glm::inverse(g_camera.get_view_matrix())*inverse_projection_matrix*move_vec);
        glm::vec3 rotation_axis = glm::normalize(glm::cross(global_move_vec, g_camera.get_look()));
        if (rotation_axis.x != rotation_axis.x ||
                rotation_axis.y != rotation_axis.y ||
                rotation_axis.z != rotation_axis.z) {
            return;
        }
        g_camera.rotate(rotation_axis);

    } else if (g_current_button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (g_mouse_pos.y > mouse_y) { /* zoom out */
            g_camera.zoom_in();
        } else if (g_mouse_pos.y < mouse_y){ /* zoom in */
            g_camera.zoom_out();
        }
    } else if (g_current_button == GLFW_MOUSE_BUTTON_MIDDLE) {
        // FIXME: middle drag
    }
    /* set new x and y */
    g_mouse_pos.x = mouse_x;
    g_mouse_pos.y = mouse_y;
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    g_mouse_pressed = (action == GLFW_PRESS);
    g_current_button = button;
    glfwGetCursorPos(window, &g_mouse_pos.x, &g_mouse_pos.y);
}

mutex renderLock;

int main(int argc, char* argv[]) {
    string window_title = "Cloth";
    if (!glfwInit()) exit(EXIT_FAILURE);
    sim = make_shared<Simulation>(&renderLock);
    glfwSetErrorCallback(ErrorCallback);

    // Ask an OpenGL 3.3 core profile context
    // It is required on OSX and non-NVIDIA Linux
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(window_width, window_height,
            &window_title[0], nullptr, nullptr);
    CHECK_SUCCESS(window != nullptr);
    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;

    CHECK_SUCCESS(glewInit() == GLEW_OK);
    glGetError();  // clear GLEW's error for it
    glfwSetKeyCallback(window, KeyCallback);
    glfwSetCursorPosCallback(window, MousePosCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSwapInterval(1);
    const GLubyte* renderer = glGetString(GL_RENDERER);  // get renderer string
    const GLubyte* version = glGetString(GL_VERSION);    // version as a string
    cout << "Renderer: " << renderer << "\n";
    cout << "OpenGL version supported:" << version << "\n";

    /* g_cloth->generate_geometry(obj_vertices, obj_faces); */
    /* g_sphere->generate_geometry(obj_vertices, obj_faces); */

    // Setup our VAO array.
    CHECK_GL_ERROR(glGenVertexArrays(kNumVaos, &g_array_objects[0]));

    // Switch to the VAO for Geometry.
    CHECK_GL_ERROR(glBindVertexArray(g_array_objects[kGeometryVao]));

    // Generate buffer objects
    CHECK_GL_ERROR(glGenBuffers(kNumVbos, &g_buffer_objects[kGeometryVao][0]));

    // Setup vertex data in a VBO.
    CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER, g_buffer_objects[kGeometryVao][kVertexBuffer]));
    // NOTE: We do not send anything right now, we just describe it to OpenGL.
    CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                sizeof(float) * obj_vertices.size() * 4, nullptr,
                GL_STATIC_DRAW));
    CHECK_GL_ERROR(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0));
    CHECK_GL_ERROR(glEnableVertexAttribArray(0));

    // Setup element array buffer.
    CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_buffer_objects[kGeometryVao][kIndexBuffer]));
    CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                sizeof(uint32_t) * obj_faces.size() * 3,
                &obj_faces[0], GL_STATIC_DRAW));

    /*
     * So far, the geometry is loaded into g_buffer_objects[kGeometryVao][*].
     * These buffers are binded to g_array_objects[kGeometryVao]
     */

    // FIXME: load the floor into g_buffer_objects[kFloorVao][*],
    //        and bind these VBO to g_array_objects[kFloorVao]
    vector<glm::vec4> floor_vertices;
    vector<glm::uvec3> floor_faces;
    floor_vertices.push_back(glm::vec4(0.0f,-2.0f,0.0f,1.0f));
    floor_vertices.push_back(glm::vec4(-10000.0f,-2.0f,-10000.0f,1.0f)); // Far Left
    floor_vertices.push_back(glm::vec4(-10000.0f,-2.0f,10000.0f,1.0f)); // Back Left
    floor_vertices.push_back(glm::vec4(10000.0f,-2.0f,10000.0f,1.0f)); // Back Right
    floor_vertices.push_back(glm::vec4(10000.0f,-2.0f,-10000.0f,1.0f)); // Far Right

    floor_faces.push_back(glm::uvec3(0, 4, 1));
    floor_faces.push_back(glm::uvec3(0, 1, 2));
    floor_faces.push_back(glm::uvec3(0, 2, 3));
    floor_faces.push_back(glm::uvec3(0, 3, 4));

    // Switch to the VAO for Geometry.
    CHECK_GL_ERROR(glBindVertexArray(g_array_objects[kFloorVao]));

    // Generate buffer objects
    CHECK_GL_ERROR(glGenBuffers(kNumVbos, &g_buffer_objects[kFloorVao][0]));

    // Setup vertex data in a VBO.
    CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER, g_buffer_objects[kFloorVao][kVertexBuffer]));
    // NOTE: We do not send anything right now, we just describe it to OpenGL.
    CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                sizeof(float) * floor_vertices.size() * 4, &floor_vertices[0],
                GL_STATIC_DRAW));
    CHECK_GL_ERROR(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0));
    CHECK_GL_ERROR(glEnableVertexAttribArray(0));

    // Setup element array buffer.
    CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_buffer_objects[kFloorVao][kIndexBuffer]));
    CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                sizeof(uint32_t) * floor_faces.size() * 3,
                &floor_faces[0], GL_STATIC_DRAW));


    // Setup vertex shader.
    GLuint vertex_shader_id = 0;
    const char* vertex_source_pointer = vertex_shader;
    CHECK_GL_ERROR(vertex_shader_id = glCreateShader(GL_VERTEX_SHADER));
    CHECK_GL_ERROR(glShaderSource(vertex_shader_id, 1, &vertex_source_pointer, nullptr));
    glCompileShader(vertex_shader_id);
    CHECK_GL_SHADER_ERROR(vertex_shader_id);

    // Setup geometry shader.
    GLuint geometry_shader_id = 0;
    const char* geometry_source_pointer = geometry_shader;
    CHECK_GL_ERROR(geometry_shader_id = glCreateShader(GL_GEOMETRY_SHADER));
    CHECK_GL_ERROR(glShaderSource(geometry_shader_id, 1, &geometry_source_pointer, nullptr));
    glCompileShader(geometry_shader_id);
    CHECK_GL_SHADER_ERROR(geometry_shader_id);

    // Setup fragment shader.
    GLuint fragment_shader_id = 0;
    const char* fragment_source_pointer = fragment_shader;
    CHECK_GL_ERROR(fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER));
    CHECK_GL_ERROR(glShaderSource(fragment_shader_id, 1, &fragment_source_pointer, nullptr));
    glCompileShader(fragment_shader_id);
    CHECK_GL_SHADER_ERROR(fragment_shader_id);

    // Let's create our program.
    GLuint program_id = 0;
    CHECK_GL_ERROR(program_id = glCreateProgram());
    CHECK_GL_ERROR(glAttachShader(program_id, vertex_shader_id));
    CHECK_GL_ERROR(glAttachShader(program_id, fragment_shader_id));
    CHECK_GL_ERROR(glAttachShader(program_id, geometry_shader_id));

    // Bind attributes.
    CHECK_GL_ERROR(glBindAttribLocation(program_id, 0, "vertex_position"));
    CHECK_GL_ERROR(glBindFragDataLocation(program_id, 0, "fragment_color"));
    glLinkProgram(program_id);
    CHECK_GL_PROGRAM_ERROR(program_id);

    // Get the uniform locations.
    GLint projection_matrix_location = 0;
    CHECK_GL_ERROR(projection_matrix_location =
            glGetUniformLocation(program_id, "projection"));
    GLint view_matrix_location = 0;
    CHECK_GL_ERROR(view_matrix_location =
            glGetUniformLocation(program_id, "view"));
    GLint light_position_location = 0;
    CHECK_GL_ERROR(light_position_location =
            glGetUniformLocation(program_id, "light_position"));

    // Setup fragment shader for the floor
    GLuint floor_fragment_shader_id = 0;
    const char* floor_fragment_source_pointer = floor_fragment_shader;
    CHECK_GL_ERROR(floor_fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER));
    CHECK_GL_ERROR(glShaderSource(floor_fragment_shader_id, 1,
                &floor_fragment_source_pointer, nullptr));
    glCompileShader(floor_fragment_shader_id);
    CHECK_GL_SHADER_ERROR(floor_fragment_shader_id);

    // FIXME: Setup another program for the floor, and get its locations.
    // Note: you can reuse the vertex and geometry shader objects
    GLuint floor_program_id = 0;
    GLint floor_projection_matrix_location = 0;
    GLint floor_view_matrix_location = 0;
    GLint floor_light_position_location = 0;

    // Let's create our program.
    CHECK_GL_ERROR(floor_program_id = glCreateProgram());
    CHECK_GL_ERROR(glAttachShader(floor_program_id, vertex_shader_id));
    CHECK_GL_ERROR(glAttachShader(floor_program_id, floor_fragment_shader_id));
    CHECK_GL_ERROR(glAttachShader(floor_program_id, geometry_shader_id));

    // Bind attributes.
    CHECK_GL_ERROR(glBindAttribLocation(floor_program_id, 0, "vertex_position"));
    CHECK_GL_ERROR(glBindFragDataLocation(floor_program_id, 0, "fragment_color"));
    glLinkProgram(floor_program_id);
    CHECK_GL_PROGRAM_ERROR(floor_program_id);


    glm::vec4 light_position = glm::vec4(10.0f, 10.0f, 10.0f, 1.0f);
    float aspect = 0.0f;
    float theta = 0.0f;

    thread simThread(takeSteps, sim);
    while (!glfwWindowShouldClose(window)) {
        // Setup some basic window stuff.
        glfwGetFramebufferSize(window, &window_width, &window_height);
        glViewport(0, 0, window_width, window_height);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDepthFunc(GL_LESS);

        // Switch to the Geometry VAO.
        CHECK_GL_ERROR(glBindVertexArray(g_array_objects[kGeometryVao]));

        // Compute the projection matrix.
        aspect = static_cast<float>(window_width) / window_height;
        projection_matrix =
            glm::perspective(glm::radians(45.0f), aspect, 0.0001f, 1000.0f);
        inverse_projection_matrix = glm::inverse(projection_matrix);

        // Compute the view matrix
        glm::mat4 view_matrix = g_camera.get_view_matrix();

        // TODO: only run thread if it's already done

        // Compute the new simulation geometry
        renderLock.lock();
        if (shouldRender.load()) {
            /* cout << "rendering" << endl; */
            sim->generate_geometry(obj_vertices, obj_faces);
            shouldRender.store(false);
        }
        renderLock.unlock();

        // Send vertices to the GPU.
        CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER,
                    g_buffer_objects[kGeometryVao][kVertexBuffer]));
        CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                    sizeof(float) * obj_vertices.size() * 4,
                    &obj_vertices[0], GL_STATIC_DRAW));

        /* Setup element array buffer. */
        CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                    g_buffer_objects[kGeometryVao][kIndexBuffer]));
        CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    sizeof(uint32_t) * obj_faces.size() * 3,
                    &obj_faces[0], GL_STATIC_DRAW));

        /* Use our program. */
        CHECK_GL_ERROR(glUseProgram(program_id));

        /* Pass uniforms in. */
        CHECK_GL_ERROR(glUniformMatrix4fv(projection_matrix_location, 1, GL_FALSE,
                    &projection_matrix[0][0]));
        CHECK_GL_ERROR(glUniformMatrix4fv(view_matrix_location, 1, GL_FALSE,
                    &view_matrix[0][0]));
        CHECK_GL_ERROR(glUniform4fv(light_position_location, 1, &light_position[0]));

        /* Draw our triangles. */
        CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES, obj_faces.size() * 3, GL_UNSIGNED_INT, 0));

        /* FIXME: Render the floor */
        /* Note: What you need to do is */
        /* 1. Switch VAO */
        CHECK_GL_ERROR(glBindVertexArray(g_array_objects[kFloorVao]));
        /* Generate buffer objects */
        CHECK_GL_ERROR(glGenBuffers(kNumVbos, &g_buffer_objects[kFloorVao][0]));
        /* 2. Switch Program */
        /* Send vertices to the GPU. */
        CHECK_GL_ERROR(glBindBuffer(GL_ARRAY_BUFFER,
                    g_buffer_objects[kFloorVao][kVertexBuffer]));
        /* NOTE: We do not send anything right now, we just describe it to OpenGL. */
        CHECK_GL_ERROR(glBufferData(GL_ARRAY_BUFFER,
                    sizeof(float) * floor_vertices.size() * 4,
                    &floor_vertices[0], GL_STATIC_DRAW));

        /* Setup element array buffer. */
        CHECK_GL_ERROR(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                    g_buffer_objects[kFloorVao][kIndexBuffer]));
        CHECK_GL_ERROR(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    sizeof(uint32_t) * floor_faces.size() * 3,
                    &floor_faces[0], GL_STATIC_DRAW));

        /* Use our program. */
        CHECK_GL_ERROR(glUseProgram(floor_program_id));

        /* 3. Pass Uniforms */
        CHECK_GL_ERROR(glUniformMatrix4fv(projection_matrix_location, 1, GL_FALSE,
                    &projection_matrix[0][0]));
        CHECK_GL_ERROR(glUniformMatrix4fv(view_matrix_location, 1, GL_FALSE,
                    &view_matrix[0][0]));
        CHECK_GL_ERROR(glUniform4fv(light_position_location, 1, &light_position[0]));

        /* 4. Call glDrawElements, since input geometry is */
        /* indicated by VAO. */
        /* Draw our triangles. */
        CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES, floor_faces.size() * 3, GL_UNSIGNED_INT, 0));

        /* Poll and swap. */
        glfwPollEvents();
        glfwSwapBuffers(window);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
