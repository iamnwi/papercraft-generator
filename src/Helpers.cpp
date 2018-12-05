#include "Helpers.h"

void VertexArrayObject::init()
{
  glGenVertexArrays(1, &id);
  check_gl_error();
}

void VertexArrayObject::bind()
{
  glBindVertexArray(id);
  check_gl_error();
}

void VertexArrayObject::free()
{
  glDeleteVertexArrays(1, &id);
  check_gl_error();
}

void VertexBufferObject::init()
{
  glGenBuffers(1,&id);
  check_gl_error();
}

void VertexBufferObject::bind()
{
  glBindBuffer(GL_ARRAY_BUFFER,id);
  check_gl_error();
}

void VertexBufferObject::free()
{
  glDeleteBuffers(1,&id);
  check_gl_error();
}

void VertexBufferObject::update(const Eigen::MatrixXf& M)
{
  assert(id != 0);
  glBindBuffer(GL_ARRAY_BUFFER, id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float)*M.size(), M.data(), GL_DYNAMIC_DRAW);
  rows = M.rows();
  cols = M.cols();
  check_gl_error();
}

void IndexBufferObject::init()
{
  glGenBuffers(1,&id);
  check_gl_error();
}

void IndexBufferObject::bind()
{
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,id);
  check_gl_error();
}

void IndexBufferObject::free()
{
  glDeleteBuffers(1,&id);
  check_gl_error();
}

void IndexBufferObject::update(const Eigen::VectorXi& V)
{
  assert(id != 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, id);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*V.size(), V.data(), GL_DYNAMIC_DRAW);
  rows = V.rows();
  cols = V.cols();
  check_gl_error();
}

bool Program::init(
  const std::string &vertex_shader_string,
  const std::string &fragment_shader_string,
  const std::string &fragment_data_name)
{
  using namespace std;
  vertex_shader = create_shader_helper(GL_VERTEX_SHADER, vertex_shader_string);
  fragment_shader = create_shader_helper(GL_FRAGMENT_SHADER, fragment_shader_string);

  if (!vertex_shader || !fragment_shader)
    return false;

  program_shader = glCreateProgram();

  glAttachShader(program_shader, vertex_shader);
  glAttachShader(program_shader, fragment_shader);

  glBindFragDataLocation(program_shader, 0, fragment_data_name.c_str());
  glLinkProgram(program_shader);

  GLint status;
  glGetProgramiv(program_shader, GL_LINK_STATUS, &status);

  if (status != GL_TRUE)
  {
    char buffer[512];
    glGetProgramInfoLog(program_shader, 512, NULL, buffer);
    cerr << "Linker error: " << endl << buffer << endl;
    program_shader = 0;
    return false;
  }

  check_gl_error();
  return true;
}

void Program::bind()
{
  glUseProgram(program_shader);
  check_gl_error();
}

GLint Program::attrib(const std::string &name) const
{
  return glGetAttribLocation(program_shader, name.c_str());
}

GLint Program::uniform(const std::string &name) const
{
  return glGetUniformLocation(program_shader, name.c_str());
}

GLint Program::bindVertexAttribArray(
        const std::string &name, VertexBufferObject& VBO) const
{
  GLint id = attrib(name);
  if (id < 0)
    return id;
  if (VBO.id == 0)
  {
    glDisableVertexAttribArray(id);
    return id;
  }
  VBO.bind();
  glEnableVertexAttribArray(id);
  glVertexAttribPointer(id, VBO.rows, GL_FLOAT, GL_FALSE, 0, 0);
  check_gl_error();

  return id;
}

void Program::free()
{
  if (program_shader)
  {
    glDeleteProgram(program_shader);
    program_shader = 0;
  }
  if (vertex_shader)
  {
    glDeleteShader(vertex_shader);
    vertex_shader = 0;
  }
  if (fragment_shader)
  {
    glDeleteShader(fragment_shader);
    fragment_shader = 0;
  }
  check_gl_error();
}

GLuint Program::create_shader_helper(GLint type, const std::string &shader_string)
{
  using namespace std;
  if (shader_string.empty())
    return (GLuint) 0;

  GLuint id = glCreateShader(type);
  const char *shader_string_const = shader_string.c_str();
  glShaderSource(id, 1, &shader_string_const, NULL);
  glCompileShader(id);

  GLint status;
  glGetShaderiv(id, GL_COMPILE_STATUS, &status);

  if (status != GL_TRUE)
  {
    char buffer[512];
    if (type == GL_VERTEX_SHADER)
      cerr << "Vertex shader:" << endl;
    else if (type == GL_FRAGMENT_SHADER)
      cerr << "Fragment shader:" << endl;
    else if (type == GL_GEOMETRY_SHADER)
      cerr << "Geometry shader:" << endl;
    cerr << shader_string << endl << endl;
    glGetShaderInfoLog(id, 512, NULL, buffer);
    cerr << "Error: " << endl << buffer << endl;
    return (GLuint) 0;
  }
  check_gl_error();

  return id;
}

void _check_gl_error(const char *file, int line)
{
  GLenum err (glGetError());

  while(err!=GL_NO_ERROR)
  {
    std::string error;

    switch(err)
    {
      case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
      case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
      case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
      case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
    }

    std::cerr << "GL_" << error.c_str() << " - " << file << ":" << line << std::endl;
    err = glGetError();
  }
}

bool loadMeshfromOFF(std::string filepath, Eigen::MatrixXf &V, Eigen::MatrixXf &C, Eigen::VectorXi &IDX) {
    // Open OFF file
    std::ifstream inFile;
    inFile.open(filepath);
    if (!inFile) {
        std::cerr << "Unable to open the OFF file";
        exit(1);   // call system to stop
    }

    // Read mesh from OFF file
    std::cout << "Reading OFF file..." << std::endl;
    int vnums, fnums, enums;
    std::string dummy;
    inFile >> dummy >> vnums >> fnums >> enums;

    // Read vertexes
    V = Eigen::MatrixXf(4, 0);
    for (int i = 0; i < vnums; i++) {
        double x,y,z;
        inFile >> x >> y >> z;
        V.conservativeResize(V.rows(), V.cols()+1);
        V.col(i) = Eigen::Vector4f(x, y, z, 1);
    }

    std::cout << vnums << " vertexes loaded" << std::endl;
    
    // Read faces and create face objects
    // Add color to each face
    std::vector<int> indices;
    for (int t = 0; t < fnums; t++) {
        int n, i,j,k;
        inFile >> n >> i >> j >> k;
        indices.push_back(i); indices.push_back(j); indices.push_back(k);
    }
    IDX.resize(indices.size());
    for (int i = 0; i < indices.size(); i++) {
      IDX(i) = indices[i];
    }
    std::cout << vnums << " vertexes loaded " << fnums << " faces loaded" << std::endl;

    inFile.close();
    return true;
}

Eigen::MatrixXf get_bounding_box(Eigen::MatrixXf V) {
  double maxx = -100, maxy = -100, maxz = -100;
  double minx = 100, miny = 100, minz = 100;
  for (int i = 0; i < V.cols(); i++) {
    double x = V.col(i)(0), y = V.col(i)(1), z = V.col(i)(2);
    if (x > maxx) maxx = x;
    if (y > maxy) maxy = y;
    if (z > maxz) maxz = z;
    if (x < minx) minx = x;
    if (y < miny) miny = y;
    if (z < minz) minz = z;
  }
  Eigen::MatrixXf bounding_box(4, 2);
  bounding_box << minx, maxx, miny, maxy, minz, maxz, 1, 1;
  std::cout << "Bouncing Box" << std::endl << bounding_box << std::endl;
  return bounding_box;
}

Eigen::MatrixXf get_bounding_box_2d(Eigen::MatrixXf V) {
  // compute bounding box
  double maxx = -100, maxy = -100;
  double minx = 100, miny = 100;
  for (int i = 0; i < V.cols(); i++) {
      double x = V.col(i)(0), y = V.col(i)(1), z = V.col(i)(2);
      if (x > maxx) maxx = x;
      if (y > maxy) maxy = y;
      if (x < minx) minx = x;
      if (y < miny) miny = y;
  }
  Eigen::MatrixXf bounding_box(2, 2);
  bounding_box << minx, maxx, miny, maxy;
  // std::cout << "minx << " " << maxx << " " << miny << " " << maxy" << std::endl;
  // std::cout << minx << " " << maxx << " " << miny << " " << maxy << std::endl;
  return bounding_box;
}

Eigen::MatrixXf get_ortho_matrix(float l, float r, float b, float t, float n, float f) {
  Eigen::Matrix4f ortho = Eigen::MatrixXf::Identity(4,4);
  ortho.col(0)(0) = 2.0/(r-l); ortho.col(3)(0) = -(r+l)/(r-l);
  ortho.col(1)(1) = 2.0/(t-b); ortho.col(3)(1) = -(t+b)/(t-b);
  ortho.col(2)(2) = 2.0/(n-f); ortho.col(3)(2) = -(n+f)/(n-f);
  return ortho;
}
Eigen::Vector3f to_3(Eigen::Vector4f X) {
  return Eigen::Vector3f(X(0), X(1), X(2));
}

Eigen::Vector4f to_4_vec(Eigen::Vector3f vec) {
  return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 0);
}

Eigen::Vector4f to_4_point(Eigen::Vector3f vec) {
  return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.0);
}

Eigen::Matrix4f mat_to_4(Eigen::Matrix3f M3) {
  Eigen::Matrix4f M4 = Eigen::MatrixXf::Identity(4, 4);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      M4.col(i)(j) = M3.col(i)(j);
    }
  }
  return M4;
}

std::string get_color_RGB(Eigen::Vector3f C) {
  std::string color_hex = "rgb(#R, #G, #B)";
  color_hex = replace_all(color_hex, "#R", std::to_string(int(C(0))));
  color_hex = replace_all(color_hex, "#G", std::to_string(int(C(1))));
  color_hex = replace_all(color_hex, "#B", std::to_string(int(C(2))));
  return color_hex;
}

std::string replace_all(std::string str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
    return str;
}

std::string get_svg_root_template() {
  std::string svg_root_template = 
  "<svg xmlns='http://www.w3.org/2000/svg' version='1.200000' width='100%' height='100%' viewBox='-1 -1 2 2' xmlns:xlink='http://www.w3.org/1999/xlink'>\
    <g transform='matrix(0.5, 0.0, 0.0, $d, 0.0, 0.3)'>\
    $TG\
    </g>\
  </svg>";
  return svg_root_template;
}

std::string get_tri_g_template() {
  std::string TRI_G_TEMPLATE = 
  "<g>\
      <polyline points='$AX,$AY $BX,$BY $CX,$CY, $AX,$AY' stroke='' stroke-width='1' fill='rgb(255, 255, 255)'/>\
      <polyline points='$AX,$AY $BX,$BY $CX,$CY, $AX,$AY' stroke='black' stroke-width='0.003' fill='none' />\
  </g>";
  return TRI_G_TEMPLATE;
}

std::string get_path_template() {
  std::string PATH_TEMPLATE = "<path d='$path' fill='none' stroke='black' stroke-width='0.003'/>";
  return PATH_TEMPLATE;
}