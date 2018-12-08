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

// #define PI 3.14159265
#define PI 3.1415926535897932384626433832795028841971693993
#define BUMPY_CUBE_OFF_PATH "../data/bumpy_cube.off"
// #define BUNNY_OFF_PATH "../data/bunny_remesh.off"
#define BUNNY_OFF_PATH "../data/bunny.off"
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
#define LEFTSUBWINDOW 0
#define RIGHTSUBWINDOW 1

bool loadMeshfromOFF(std::string filepath, Eigen::MatrixXd &V, Eigen::MatrixXd &C, Eigen::VectorXi &IDX);
Eigen::MatrixXd get_bounding_box(Eigen::MatrixXd V);
Eigen::MatrixXd get_bounding_box_2d(Eigen::MatrixXd V);
Eigen::Vector3d to_3(Eigen::Vector4d X);
Eigen::Matrix4d LookAtRH( Eigen::Vector3d eye, Eigen::Vector3d target, Eigen::Vector3d up );
Eigen::MatrixXd get_ortho_matrix(double l, double r, double b, double t, double n, double f);
Eigen::Vector4d to_4_vec(Eigen::Vector3d vec);
Eigen::Vector4d to_4_point(Eigen::Vector3d vec);
Eigen::Matrix4d mat_to_4(Eigen::Matrix3d M);
Eigen::Vector3d get_vertical_vec(Eigen::Vector3d vech3, Eigen::Vector3d rotAixs);
Eigen::Matrix4d get_rotate_mat(double rad, Eigen::Vector3d edgeA, Eigen::Vector3d edgeB);
Eigen::Matrix4d get_rotate_mat(double rotDot, double rotSign, Eigen::Vector3d edgeA, Eigen::Vector3d edgeB);
Eigen::VectorXf v_to_float(Eigen::VectorXd in);
Eigen::MatrixXf m_to_float(Eigen::MatrixXd in);

std::string replace_all(std::string str, const std::string& from, const std::string& to);
std::string get_tri_g_template();
std::string get_svg_root_template();
std::string get_path_template();
