#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>

#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#  undef DrawText
#endif

#ifndef __APPLE__
#  define GLEW_STATIC
#  include <GL/glew.h>
#endif

#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   define __gl_h_ /* Prevent inclusion of the old gl.h */
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif

class VertexArrayObject
{
public:
    unsigned int id;

    VertexArrayObject() : id(0) {}

    // Create a new VAO
    void init();

    // Select this VAO for subsequent draw calls
    void bind();

    // Release the id
    void free();
};

class VertexBufferObject
{
public:
    typedef unsigned int GLuint;
    typedef int GLint;

    GLuint id;
    GLuint rows;
    GLuint cols;

    VertexBufferObject() : id(0), rows(0), cols(0) {}

    // Create a new empty VBO
    void init();

    // Updates the VBO with a matrix M
    void update(const Eigen::MatrixXf& M);

    // Select this VBO for subsequent draw calls
    void bind();

    // Release the id
    void free();
};

class IndexBufferObject
{
public:
    typedef unsigned int GLuint;
    typedef int GLint;

    GLuint id;
    GLuint rows;
    GLuint cols;

    IndexBufferObject() : id(0), rows(0), cols(0) {}

    // Create a new empty VBO
    void init();

    // Updates the VBO with a vector V
    void update(const Eigen::VectorXi& V);

    // Select this VBO for subsequent draw calls
    void bind();

    // Release the id
    void free();
};

// This class wraps an OpenGL program composed of two shaders
class Program
{
public:
  typedef unsigned int GLuint;
  typedef int GLint;

  GLuint vertex_shader;
  GLuint fragment_shader;
  GLuint program_shader;

  Program() : vertex_shader(0), fragment_shader(0), program_shader(0) { }

  // Create a new shader from the specified source strings
  bool init(const std::string &vertex_shader_string,
  const std::string &fragment_shader_string,
  const std::string &fragment_data_name);

  // Select this shader for subsequent draw calls
  void bind();

  // Release all OpenGL objects
  void free();

  // Return the OpenGL handle of a named shader attribute (-1 if it does not exist)
  GLint attrib(const std::string &name) const;

  // Return the OpenGL handle of a uniform attribute (-1 if it does not exist)
  GLint uniform(const std::string &name) const;

  // Bind a per-vertex array attribute
  GLint bindVertexAttribArray(const std::string &name, VertexBufferObject& VBO) const;

  GLuint create_shader_helper(GLint type, const std::string &shader_string);

};

// From: https://blog.nobel-joergensen.com/2013/01/29/debugging-opengl-using-glgeterror/
void _check_gl_error(const char *file, int line);

///
/// Usage
/// [... some opengl calls]
/// glCheckError();
///
#define check_gl_error() _check_gl_error(__FILE__,__LINE__)

#endif

#define PI 3.14159265
#define BUMPY_CUBE_OFF_PATH "../data/bumpy_cube.off"
#define BUNNY_OFF_PATH "../data/bunny_remesh.off"
#define CUBE_OFF_PATH "../data/cube.off"
// #define CUBE_OFF_PATH "../data/cube_desk.off"
#define BALL_OFF_PATH "../data/ball.off"
#define CONE_OFF_PATH "../data/cone.off"
#define FOX_OFF_PATH "../data/fox.off"
#define WIREFRAME 0
#define FLAT_SHADING 1
#define PHONG_SHADING 2
#define SCREEN_Z -2.0
#define SELCET_COLOR BLUE
#define RENDER_MODE_NUMBER 3
#define ORTHO -1
#define PERSPECT 1
#define DIST_MAX 10000.
#define CILCK_ACTION 1
#define RAY_ACTION 2
#define ESP 1e-6

bool loadMeshfromOFF(std::string filepath, Eigen::MatrixXf &V, Eigen::MatrixXf &C, Eigen::VectorXi &IDX);
Eigen::MatrixXf get_bounding_box(Eigen::MatrixXf V);
Eigen::Vector3f to_3(Eigen::Vector4f X);
Eigen::Matrix4f LookAtRH( Eigen::Vector3f eye, Eigen::Vector3f target, Eigen::Vector3f up );
Eigen::MatrixXf get_ortho_matrix(float l, float r, float b, float t, float n, float f);
Eigen::Vector4f to_4_vec(Eigen::Vector3f vec);
Eigen::Vector4f to_4_point(Eigen::Vector3f vec);

std::string replace_all(std::string str, const std::string& from, const std::string& to);
std::string get_tri_g_template();
std::string get_svg_root_template();
std::string get_path_template();
