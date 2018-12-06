// This example is heavily based on the tutorial at https://open.gl

#include <iostream>
#include <math.h>
#include <vector>
#include <list> 
#include <algorithm>
#include <fstream>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <cassert>

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

// Timer
#include <chrono>

// Contains the vertex positions
Eigen::MatrixXf V(2,3);

// Contains the per-vertex color
Eigen::MatrixXf C(3,3);

// The view matrix
// Eigen::MatrixXf ViewMat(4,4);
Eigen::MatrixXf ProjectMat(4,4);

// Color, order in Key1 to Key9
Eigen::Vector3i RED(255, 0, 0);
Eigen::Vector3i GREEN(0, 255, 0);
Eigen::Vector3i BLUE(0, 0, 255); // Selected color
Eigen::Vector3i YELLOW(255, 255, 0);
Eigen::Vector3i BLACK(0, 0, 0);
Eigen::Vector3i WHITE(255, 255, 255);
Eigen::Vector3i GREY(128,128,128);
Eigen::Vector3i ORANGE(255, 165, 0);
Eigen::Vector3i PURPLE(160, 32, 240);
Eigen::Vector3i DIMGREY(105,105,105);
Eigen::Vector3i LIGHTGREY(200,200,200);
std::vector<Eigen::Vector3i> colors = {RED, GREEN, YELLOW, WHITE, LIGHTGREY, ORANGE, PURPLE};

// Flags
float pre_aspect_ratio = -1;
bool pre_aspect_is_x = false;
bool drag = false;
float hit_dist;
Eigen::Vector4f pre_cursor_point;

class Light {
    public:
        Eigen::Vector4f position;
        Eigen::Vector3f intensity; // default (1,1,1)
        Light(Eigen::Vector4f position, Eigen::Vector3f intensity = Eigen::Vector3f(1,1,1)) {
            this->position = position; this->intensity = intensity;
        }
};
class Camera {
    public:
        Eigen::Matrix4f flatViewMat;
        Eigen::Matrix4f ViewMat;
        Eigen::Matrix4f ortho_mat;
        Eigen::Matrix4f perspect_mat;
        Eigen::Vector3f position;
        Eigen::Vector3f global_up;
        Eigen::Vector3f up;
        Eigen::Vector3f right;
        Eigen::Vector3f forward;
        Eigen::Vector3f target;

        // bounding box
        float n, f, t, b, r, l;
        int project_mode;
        float theta;
        int phi, zeta;
        float radius;

        Camera(GLFWwindow* window, Eigen::Vector3f position=Eigen::Vector3f(0.0, 0.0, 3.0), int project_mode = ORTHO) {
            // set GLOBAL UP to y-axis as default, set look at target as origin(0,0,0)
            this->global_up = Eigen::Vector3f(0.0, 1.0, 0.0);
            this->n = 2.0; this->f = 100.0;
            // FOV angle is hardcoded to 60 degrees
            this->theta = (PI/180) * 60;
            // trackball
            this->target = Eigen::Vector3f(0.0, 0.0, 0.0);
            this->phi = 0;
            this->zeta = 0;
            this->radius = 3.0;

            this->flatViewMat << 
            1.0, 0.0, 0.0, -position(0),
            0.0, 1.0, 0.0, -position(1),
            0.0, 0.0, 1.0, -position(2),
            0.0, 0.0, 0.0, 1.0;
        
            this->update_camera_pos();
            this->look_at(window);
            this->project_mode = project_mode;
        }
        void update_camera_pos() {
            float rphi = (PI/180.0)*this->phi, rzeta = (PI/180.0)*this->zeta;
            float y = this->radius * sin(rphi);
            float x = this->radius * sin(rzeta) * cos(rphi);
            float z = this->radius * cos(rzeta) * cos(rphi);

            this->position = Eigen::Vector3f(x, y, z);
        }
        void rotate(int dPhi, int dZeta) {
            this->phi = (this->phi+dPhi+360) % 360;
            this->zeta = (this->zeta+dZeta+360) % 360;
            this->update_camera_pos();
        }
        void zoom(double factor) {
            this->radius *= factor;
            this->update_camera_pos();
        }
        void reset() {
            this->phi = 0;
            this->zeta = 0;
            this->update_camera_pos();
        }
        void look_at(GLFWwindow* window, Eigen::Vector3f target = Eigen::Vector3f(0.0, 0.0, 0.0)) {
            this->forward = (this->position - this->target).normalized();
            // special case when forward is parallel to global up
            float dotVal = this->global_up.dot(this->forward);
            if (this->phi == 90 || this->phi == 270) {
                this->right = (this->up.cross(this->forward).normalized());
            }
            else if (this->phi > 90 && this->phi < 270) {
                this->right = (-this->global_up.cross(this->forward).normalized());
            }
            else {
                this->right = (this->global_up.cross(this->forward).normalized());
            }
            this->up = this->forward.cross(this->right);

            auto w = this->forward, u = this->right, v = this->up;
            Eigen::Matrix4f LOOK;
            LOOK <<
            u(0), u(1), u(2), 0.0,
            v(0), v(1), v(2), 0.0,
            w(0), w(1), w(2), 0.0,
            0.0,   0.0, 0.0,  1.0;

            Eigen::Matrix4f AT;
            AT <<
            1.0, 0.0, 0.0, -this->position(0),
            0.0, 1.0, 0.0, -this->position(1),
            0.0, 0.0, 1.0, -this->position(2),
            0.0, 0.0, 0.0, 1.0;

            this->ViewMat = LOOK * AT;

            this->update_project_mat(window);
        }
        Eigen::Vector3f to_world_point(Eigen::Vector3f screen_point) {
            Eigen::Vector3f global_x, global_y, global_z;
            global_x << 1.0, 0.0, 0.0;
            global_y << 0.0, 1.0, 0.0;
            global_z << 0.0, 0.0, 1.0;
            double xworld = screen_point.dot(global_x);
            double yworld = screen_point.dot(global_y);
            double zworld = screen_point.dot(global_z);
            return Eigen::Vector3f(xworld, yworld, zworld);
        }
        void switch_project_mode() {
            this->project_mode *= -1;
        }
        void update_project_mat(GLFWwindow* window) {
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            width /= 2;
            // height /= 2;
            double aspect = (double)width/(double)height;
            this->t = tan(theta/2) * std::abs(n);
            this->b = -this->t;
            this->r = aspect * this->t;
            this->l = -this->r;

            this->ortho_mat << 
            2.0/(r-l), 0.0, 0.0, -(r+l)/(r-l),
            0.0, 2.0/(t-b), 0.0, -(t+b)/(t-b),
            0.0, 0.0, -2.0/(f-n), -(f+n)/(f-n),
            // 0.0, 0.0, -2.0/(f-n), -(n+f)/(n-f),
            0.0, 0.0, 0.0, 1.0;

            this->perspect_mat <<
            2*n/(r-l), 0.,      (r+l)/(r-l),    0.,
            0., (2*n)/(t-b),    (t+b)/(t-b),    0.,
            0., 0.,             -(f+n)/(f-n), (-2*f*n)/(f-n),
            0., 0.,             -1.,            0;
        }
        Eigen::Matrix4f get_project_mat() {
            if (this->project_mode == ORTHO) return this->ortho_mat;
            return this->perspect_mat;
        }
};
class RayTracer {
    public:
        std::vector<Light*> lights;
        void add_light(Eigen::Vector4f position) {
            this->lights.push_back(new Light(position));
        }
        Eigen::Vector3f get_diffuse_color(Eigen::Vector4f vpos, Eigen::Vector4f meshNormals, Eigen::Vector3f color, Eigen::MatrixXf ModelMat) {
            auto light = this->lights[0];
            // ambinet
            float ambientStrength = 0.01;
            Eigen::Vector3f ambient = ambientStrength * light->intensity;
            //diffuse
            Eigen::Matrix3f diffuse_mat;
            diffuse_mat <<  color(0), 0, 0,
                            0, color(1), 0,
                            0, 0, color(2);
            // Eigen::Matrix4f normalMatrix = (ModelMat.inverse()).transpose();
            // Eigen::Vector4f normal = (normalMatrix * meshNormals).normalized();
            Eigen::Vector4f normal = meshNormals;
            Eigen::Vector4f surfaceToLight = light->position - vpos;
            // float brightness = normal.dot(surfaceToLight) / (surfaceToLight.norm() * surfaceToLight.norm());
            float brightness = normal.dot(surfaceToLight);
            brightness = fmax(brightness, 0.);
            Eigen::Vector3f diffuse = diffuse_mat * light->intensity * brightness;
            Eigen::Vector3f result = ambient + diffuse;
            return result;
        }
};
class BezierCurve {
    public:
        Eigen::MatrixXf V;
        VertexArrayObject VAO;
        VertexBufferObject VBO_P;

        Eigen::MatrixXf curve_points;
        VertexArrayObject VAO_C;
        VertexBufferObject VBO_CP;

        int nxt_curve_pt;
        bool anime_finish;

        BezierCurve() {
            this->V.resize(4, 0);
            // Create a VAO
            this->VAO.init();
            this->VAO.bind();
            // Initialize the VBO with the vertices data
            this->VBO_P.init();

            this->curve_points.resize(4, 0);
            // Create a VAO
            this->VAO_C.init();
            this->VAO_C.bind();
            // Initialize the VBO with the vertices data
            this->VBO_CP.init();

            nxt_curve_pt = 0;
            anime_finish = true;
        }
        void insert_point(double x, double y, double z) {
            // this->control_points.push_back(new Circle(x, y, z));
            this->V.conservativeResize(V.rows(), V.cols()+1);
            this->V.col(V.cols()-1) << x, y, z, 1.;
            this->VBO_P.update(this->V);
            this->update_BP();
        }
        void update_BP() {
            if (this->V.cols() <= 0) return;
            double delta = 0.01;
            int line_amount = 1/delta;
            this->curve_points.resize(curve_points.rows(), line_amount+1);
            for (int i = 0; i <= line_amount; i += 1) {
                double t = i*delta;
                Eigen::Vector4f p = get_BP_at(t);
                this->curve_points.col(i) << Eigen::Vector4f(p(0), p(1), p(2), 1.);
                // this->curve_points.col(i) << p;
            }
            this->VBO_CP.update(this->curve_points);
        }
        Eigen::Vector4f get_BP_at(double t) {
            Eigen::MatrixXf CP = this->V;
            return get_BP(t, CP, CP.cols());
        }
        Eigen::Vector4f get_BP(double t, Eigen::MatrixXf CP, int n) {
            if (n == 1) return CP.col(0);
            for (int i = 0; i < n-1; i++) {
                CP.col(i) = (1-t)*CP.col(i) + t*CP.col(i+1);
            }
            return get_BP(t, CP, n-1);
        }
        bool translate(Eigen::Vector4f delta, int point_idx) {
            if (point_idx >= this->V.cols()) return false;
            this->V.col(point_idx) += delta;
            // this->control_points[point_idx]->translate(delta(0), delta(1), delta(2));
            this->VBO_P.update(this->V);
            this->update_BP();
            return true;
        }
};
class Mesh {
    public:
        Eigen::Vector3f color;

        Eigen::Vector4f centroid;
        double r, s, tx, ty;
        Eigen::Vector4f normal;

        std::map<int, Eigen::Vector3f> vid2v;
        std::map<int, Eigen::Vector3f> vid2fv;
        std::vector<int> vids;
        std::vector< std::pair< int, std::pair<int,int> > > nebMeshes;
        int id;

        VertexArrayObject VAO;
        VertexBufferObject VBO_P;

        Mesh() {}
        Mesh(int id, Eigen::MatrixXf V, int v1, int v2, int v3, Eigen::Vector3i color=LIGHTGREY) {
            this->id = id;
            this->color = color.cast<float>()/255.;
            this->vids.push_back(v1); this->vids.push_back(v2); this->vids.push_back(v3);
            this->vid2v[v1] = V.col(v1); this->vid2v[v2] = V.col(v2); this->vid2v[v3] = V.col(v3);
            // init flat position
            Eigen::Vector3f zero = Eigen::VectorXf::Zero(3);
            this->vid2fv[v1] = zero; this->vid2fv[v2] = zero; this->vid2fv[v3] = zero;

            this->VAO.init();
            this->VAO.bind();
            this->VBO_P.init();
            this->updateVBOP();
        }
        void updateVBOP() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3f fV;
            fV << vid2fv[v1], vid2fv[v2], vid2fv[v3];
            this->VBO_P.update(fV);
        }
        Eigen::Matrix3f getFlatV() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3f fV;
            fV << vid2fv[v1], vid2fv[v2], vid2fv[v3];
            return fV;
        }
        Eigen::Matrix3f getV() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3f V;
            V << vid2v[v1], vid2v[v2], vid2v[v3];
            return V;
        }
        Mesh(Eigen::MatrixXf V, Eigen::MatrixXf bounding_box, Eigen::Vector3i color=LIGHTGREY) {
            this->r = 0; this->s = 1; this->tx = 0; this->ty = 0;
            this->color = color.cast<float>();
            
            auto a = to_3(V.col(0)), b = to_3(V.col(1)), c = to_3(V.col(2));
            auto tmp = ((b-a).cross(c-a)).normalized();
            this->normal = Eigen::Vector4f(tmp(0), tmp(1), tmp(2), 0.0);
            // Computer the barycenter(centroid) of the Mesh, alphea:beta:gamma = 1:1:1
            Eigen::Vector4f A = V.col(0), B = V.col(1), C = V.col(2);
            this->centroid = (1.0/3)*A + (1.0/3)*B + (1.0/3)*C;
        }
        bool getIntersection(Eigen::Vector4f ray_origin, Eigen::Vector4f ray_direction, Eigen::Vector4f &intersection, Eigen::MatrixXf V) {
            // solve equation:     e + td = a + u(b-a) + v(c-a)
            //                     (a-b)u + (a-c)v + dt = a-e
            Eigen::Vector4f a = V.col(0), b = V.col(1), c = V.col(2);
            Eigen::Vector4f d = ray_direction, e = ray_origin;
            double ai = (a-b)(0), di = (a-c)(0), gi = (d)(0);
            double bi = (a-b)(1), ei = (a-c)(1), hi = (d)(1);
            double ci = (a-b)(2), fi = (a-c)(2), ii = (d)(2);
            double ji = (a-e)(0), ki = (a-e)(1), li = (a-e)(2);
            double M = ai*(ei*ii-hi*fi)+bi*(gi*fi-di*ii)+ci*(di*hi-ei*gi);

            double xt = -(fi*(ai*ki-ji*bi)+ei*(ji*ci-ai*li)+di*(bi*li-ki*ci))/M;
            if (xt <= 0) return false;

            double xu = (ji*(ei*ii-hi*fi)+ki*(gi*fi-di*ii)+li*(di*hi-ei*gi))/M;
            if (xu < 0 || xu > 1) return false;

            double xv = (ii*(ai*ki-ji*bi)+hi*(ji*ci-ai*li)+gi*(bi*li-ki*ci))/M;
            
            // intersection inside the Mesh
            intersection = ray_origin + xt*ray_direction;
            intersection(3) = 1.0;

            if (xv < 0 || xv+xu > 1) return false;
            return true;
        }
};
class Node {
    public:
        double weight;
        std::pair<int, int> edge;
        int parentMeshId, meshId;

        Node(double weight, std::pair<int, int> edge, int parentMeshId, int meshId) {
            this->weight = weight;
            this->edge = edge;
            this->parentMeshId = parentMeshId;
            this->meshId = meshId;
        }
};
class CompareWeight {
    public:
        bool operator()(Node a, Node b) {
            return a.weight < b.weight;
        }
};
class Grid {
    public:
        ~Grid() {
            std::cout << "enter Grid destructor" << std::endl;
            std::cout << "exit Grid destructor" << std::endl;
        }
        double sizex, sizey;
        std::map<int, std::map<int, std::vector<int> > > rows;
        Grid(double sizex = 0.03, double sizey = 0.03) {
            this->sizex = sizex; this->sizey = sizey;
        }
        void addItem(Mesh &mesh) {
            Eigen::MatrixXf boundingBox = get_bounding_box_2d(mesh.getFlatV());
            double minx = boundingBox.col(0)(0), maxx = boundingBox.col(1)(0);
            double miny = boundingBox.col(0)(1), maxy = boundingBox.col(1)(1);
            double x = minx;
            while (x < maxx+sizex) {
                double y = miny;
                while (y < maxy+sizey) {
                    int r, c;
                    getCellIdx(x, y, r, c);
                    if (std::find(rows[r][c].begin(), rows[r][c].end(), mesh.id) == rows[r][c].end())
                        rows[r][c].push_back(mesh.id);
                    y += sizey;
                }
                x += sizex;
            }
        }
        void getCellIdx(double x, double y, int &r, int &c) {
            r = int(x/sizex);
            c = int(y/sizey);
        }
        std::set<int> getNearMeshes(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C) {
            // compute bounding box
            Eigen::Matrix3f V;
            V << A, B, C;
            Eigen::MatrixXf boundingBox = get_bounding_box_2d(V);
            double minx = boundingBox.col(0)(0), maxx = boundingBox.col(1)(0);
            double miny = boundingBox.col(0)(1), maxy = boundingBox.col(1)(1);
            double x = minx;
            std::set<int> nearMeshes;
            while (x < maxx+sizex) {
                double y = miny;
                while (y < maxy+sizey) {
                    int r, c;
                    getCellIdx(x, y, r, c);
                    for (int meshId: rows[r][c]) {
                        nearMeshes.insert(meshId);
                    }
                    y += sizey;
                }
                x += sizex;
            }
            return nearMeshes;
        }
};
class FlattenObject {
    public:
        std::map<int, Mesh> meshes;
        std::map<std::pair<int, int>, std::vector<int>> edge2meshes;
        std::map<std::pair<int, int>, double> edge2weight;
        Grid* grid;

        Eigen::MatrixXf V;
        Eigen::MatrixXf fV;
        Eigen::VectorXi IDX;

        VertexArrayObject VAO;
        VertexBufferObject VBO_P;
        IndexBufferObject IBO_IDX;

        Eigen::MatrixXf ModelMat;
        Eigen::MatrixXf T_to_ori;
        Eigen::Vector4f barycenter;

        void addEdge(int v1, int v2, int meshId) {
            auto edge = v1 < v2? std::make_pair(v1, v2) : std::make_pair(v2, v1);
            edge2meshes[edge].push_back(meshId);
            if (edge2weight.find(edge) == edge2weight.end())
                edge2weight[edge] = (V.col(v1)-V.col(v2)).norm();
        }
        void addNebMeshes(int v1, int v2, int meshId) {
            auto edge = v1 < v2? std::make_pair(v1, v2) : std::make_pair(v2, v1);
            for (int nebMeshId: edge2meshes[edge]) {
                if (nebMeshId != meshId) {
                    meshes[meshId].nebMeshes.push_back(std::make_pair(nebMeshId, edge));
                }
            }
        }
        bool flattenFirst(int meshId, std::set<int> &flatten) {
            Mesh &mesh = meshes[meshId];
            int v1 = mesh.vids[0], v2 = mesh.vids[1], v3 = mesh.vids[2];
            float v1v2Len = (mesh.vid2v[v1] - mesh.vid2v[v2]).norm();
            mesh.vid2fv[v1] = Eigen::Vector3f(0., 0., 0.);
            mesh.vid2fv[v2] = Eigen::Vector3f(0., v1v2Len, 0.);
            Eigen::Vector3f flatPos;
            if (!flattenVertex(meshId, v3, v1, v2, mesh.vid2fv[v1], mesh.vid2fv[v2], flatPos, flatten)) {
                return false;
            }
            mesh.vid2fv[v3] = flatPos;
            return true;
        }
        FlattenObject(Eigen::MatrixXf V, Eigen::VectorXi IDX, std::vector<bool> &meshFlattened) {
            V.conservativeResize(3, V.cols());
            this->V = V;
            this->fV.resize(3, 0);

            // create V and F matrix
            std::cout << "create meshes" << std::endl;
            // std::vector<Mesh*> meshes;
            for (int i = 0; i < IDX.rows(); i += 3) {
                int meshId = i/3;
                // std::cout << "check id " << i/3 << std::endl;
                if (meshFlattened[i/3]) continue;
                // std::cout << "ok id " << i/3 << std::endl;
                int v1 = IDX(i), v2 = IDX(i+1), v3 = IDX(i+2);
                meshes[meshId] = Mesh(meshId, V, v1, v2, v3, WHITE);
                // add edge
                addEdge(v1, v2, meshId);
                addEdge(v2, v3, meshId);
                addEdge(v1, v3, meshId);
            }

            std::cout << "created meshes and edge to meshes" << std::endl;
            // std::cout << "face #: " << meshes.size() << std::endl;
            // std::cout << "edge #: " << edge2meshes.size() << std::endl;

            // add edge field to mesh objects
            for (auto it: meshes) {
                int meshId = it.first;
                Mesh &mesh = it.second;
                int v1 = mesh.vids[0], v2 = mesh.vids[1], v3 = mesh.vids[2];
                addNebMeshes(v1, v2, meshId);
                addNebMeshes(v2, v3, meshId);
                addNebMeshes(v1, v3, meshId);
                // assert(mesh->nebMeshes.size() == 3);
            }
            // std::cout << "created meshes to nebs" << std::endl;

            // new a Regular Grid to boost the overlap checking process.
            grid = new Grid();

            // maximal spaning tree
            std::priority_queue<Node, std::vector<Node>, CompareWeight> pq;
            std::set<int> flattened;
            std::vector<double> dist(IDX.rows()/3, 0.);

            // flat first mesh
            int firstMeshId = meshes.begin()->first;
            flattenFirst(firstMeshId, flattened);
            flattened.insert(firstMeshId);
            grid->addItem(meshes[firstMeshId]);
            dist[firstMeshId] = DIST_MAX;
            pq.push(Node(DIST_MAX, std::make_pair(0,0), 0, firstMeshId));
            std::cout << "flattened first mesh" << std::endl;
            // std::cout << "mesh flat V" << std::endl;
            std::cout << meshes[firstMeshId].getFlatV() << std::endl;
            
            // max spanning tree, prime algorithm
            while (!pq.empty()) {
                auto node = pq.top();
                pq.pop();
                Mesh &curMesh = meshes[node.meshId];
                for(auto meshNedge: curMesh.nebMeshes) {
                    int nebMeshId = meshNedge.first;
                    auto edge = meshNedge.second;
                    if (flattened.find(nebMeshId) == flattened.end() && edge2weight[edge] > dist[nebMeshId]) {
                        dist[nebMeshId] = edge2weight[edge];
                        pq.push(Node(edge2weight[edge], edge, curMesh.id, nebMeshId));
                    }
                }
                // pop out all meshes that is flatted or cannot be flatted in this island
                while (!pq.empty() && (flattened.find(pq.top().meshId) != flattened.end() || !flattenMesh(pq.top().parentMeshId, pq.top().meshId, pq.top().edge, flattened))) {
                    pq.pop();
                }
                if (!pq.empty()) {
                    node = pq.top();
                    flattened.insert(node.meshId);
                    grid->addItem(meshes[node.meshId]);
                }
            }

            for (int meshId: flattened) {
                meshFlattened[meshId] = true;
                Eigen::Matrix3f flatV = meshes[meshId].getFlatV();
                this->fV.conservativeResize(3, fV.cols()+3);
                int last = this->fV.cols();
                this->fV.col(last-3) = flatV.col(0);
                this->fV.col(last-2) = flatV.col(1);
                this->fV.col(last-1) = flatV.col(2);
            }

            // update flat position to VBO, compute bounding box
            this->VAO.init();
            this->VAO.bind();
            this->VBO_P.init();
            this->VBO_P.update(this->fV);

            // init model fields
            this->ModelMat = Eigen::MatrixXf::Identity(4,4);
            this->T_to_ori = Eigen::MatrixXf::Identity(4,4);
            this->barycenter = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
        }
        
        bool flattenMesh(int preMeshId, int meshId, std::pair<int, int> edge, std::set<int> &flattened) {
            // find the remaining non-flattened vertex
            // std::cout << "enter flattenMesh" << std::endl;
            Mesh &preMesh = meshes[preMeshId];
            Mesh &mesh = meshes[meshId];
            int fv1 = edge.first, fv2 = edge.second;
            int v3;
            for (int vid: mesh.vids) {
                if (vid != fv1 && vid != fv2) {
                    v3 = vid;
                    break;
                }
            }

            // flatten the remaining vertex v3 according to the flat position of v1 and v2
            Eigen::Vector3f fv3Pos;
            if (!flattenVertex(meshId, v3, fv1, fv2, preMesh.vid2fv[fv1], preMesh.vid2fv[fv2], fv3Pos, flattened))
                return false;

            // get flat v1 and flat v2 from pre Mesh
            mesh.vid2fv[fv1] = preMesh.vid2fv[fv1];
            mesh.vid2fv[fv2] = preMesh.vid2fv[fv2];
            mesh.vid2fv[v3] = fv3Pos;

            // std::cout << "exit flattenMesh" << std::endl;
            return true;
        }

        // compute the flat position of v3 according to the flat position of v1 and v2
        // check overlap
        bool flattenVertex(int meshId, int v3, int v1, int v2, Eigen::Vector3f fv1Pos, Eigen::Vector3f fv2Pos, Eigen::Vector3f &fv3Pos, std::set<int> &flattened) {
            Mesh &mesh = meshes[meshId];
            Eigen::Vector3f flat1, flat2;
            Eigen::Vector3f v1Pos = mesh.vid2v[v1];
            Eigen::Vector3f v2Pos = mesh.vid2v[v2];
            Eigen::Vector3f v3Pos = mesh.vid2v[v3];
            float v1v2Len = (v2Pos - v1Pos).norm();
            float v1v3Len = (v3Pos - v1Pos).norm();
            float v2v3Len = (v3Pos - v2Pos).norm();

            float S = (v2Pos-v1Pos).cross(v3Pos-v1Pos).norm()/2;
            float h = 2*S / v1v2Len;
            float b = sqrt(v1v3Len*v1v3Len - h*h);
            float c = sqrt(v2v3Len*v2v3Len - h*h);
            if (c > v1v2Len && c > b) b = -b;
            
            // compute flat pos of H and flat pos of v3
            Eigen::Vector3f fH = ((v1v2Len-b)/v1v2Len)*fv1Pos + (b/v1v2Len)*fv2Pos;
            Eigen::Vector3f fv1v2 = fv2Pos - fv1Pos;
            Eigen::Vector3f flatDir = Eigen::Vector3f(-fv1v2.y(), fv1v2.x(), 0.).normalized();
            flat1 = fH + h * flatDir;
            flat2 = fH + h * (-flatDir);

            // check overlap
            if (!overlap(flat1, fv1Pos, fv2Pos, flattened)) {
                fv3Pos = flat1;
                return true;
            }
            else {
                if (!overlap(flat2, fv1Pos, fv2Pos, flattened)) {
                    fv3Pos = flat2;
                    return true;
                }
            }
            return false;
        }

        bool overlap(Eigen::Vector3f flatPos, Eigen::Vector3f fv1Pos, Eigen::Vector3f fv2Pos, std::set<int> &flattened) {
            // check if any vertices of a flat Triangle inside the other flat Triangle
            // get all near meshes and combine them to one vector
            std::set<int> nearMeshes = grid->getNearMeshes(flatPos, fv1Pos, fv2Pos);
            for (int meshId: nearMeshes) {
                Eigen::Matrix3f meshfV = meshes[meshId].getFlatV();
                if (isInside(flatPos, meshfV)) return true;
                Eigen::Vector3f center = (flatPos+fv1Pos+fv2Pos)/3.;
                if (isInside(center, meshfV)) return true;

                Eigen::Matrix3f curMeshfV;
                curMeshfV << flatPos, fv1Pos, fv2Pos;
                if (isInside(meshfV.col(0), curMeshfV)) return true;
                if (isInside(meshfV.col(1), curMeshfV)) return true;
                if (isInside(meshfV.col(2), curMeshfV)) return true;
                center = (meshfV.col(0)+meshfV.col(1)+meshfV.col(2))/3.;
                if (isInside(center, curMeshfV)) return true;
            }

            // check line intersection
            for (int meshId: nearMeshes) {
                if (lineCross(flatPos, fv1Pos, meshId)) return true;
                if (lineCross(flatPos, fv2Pos, meshId)) return true;
            }
            return false;
        }
        bool lineCross(Eigen::Vector3f a, Eigen::Vector3f b, int meshId) {
            Mesh &mesh = meshes[meshId];
            int ov1, ov2, ov3;
            ov1 = mesh.vids[0]; ov2 = mesh.vids[1]; ov3 = mesh.vids[2];
            if (lineCross(a, b, mesh.vid2fv[ov1], mesh.vid2fv[ov2])) return true;
            if (lineCross(a, b, mesh.vid2fv[ov2], mesh.vid2fv[ov3])) return true;
            if (lineCross(a, b, mesh.vid2fv[ov1], mesh.vid2fv[ov3])) return true;
            return false;
        }
        bool lineCross(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f d) {
            // overlap?
            if (((a-c).norm() < ESP || (a-d).norm() < ESP) && ((b-c).norm() < ESP || (b-d).norm() < ESP)) {
                return false;
            }
            // parallel?
            Eigen::Vector3f AB = b-a;
            Eigen::Vector3f CD = d-c;
            if (AB.cross(CD).norm() - 0. < ESP) {
                return false;
            }

            // get intersection
            Eigen::Matrix2d M;
            Eigen::Vector2d R;
            M << a.x()-b.x(), d.x()-c.x(), a.y()-b.y(),  d.y()-c.y();
            R << d.x()-b.x(), d.y()-b.y();
            Eigen::Vector2d x = M.colPivHouseholderQr().solve(R);

            // within range? 0 <= x <= 1
            return x(0) > ESP && x(0) < 1.0-ESP && x(1) > ESP && x(1) < 1.0-ESP;
        }
        bool isInside(Eigen::Vector3f flatPos, Eigen::Matrix3f meshfV) {
            Eigen::Vector3f a, b, c;
            a = meshfV.col(0); b = meshfV.col(1); c = meshfV.col(2);

            Eigen::Matrix3d M;
            Eigen::Vector3d R;
            M << a(0),b(0),c(0),  a(1),b(1),c(1), 1,1,1;
            R << flatPos(0), flatPos(1), 1;
            Eigen::Vector3d x = M.colPivHouseholderQr().solve(R);

            return x(0)-0. > ESP && x(1)-0. > ESP && x(2)-0. > ESP;
        }
        bool hit(Eigen::Vector4f ray_origin, Eigen::Vector4f ray_direction, float &dist) {
            bool intersected = false;
            // // for (int meshId : this->meshes) {
            //     // Eigen::Vector4f intersection;
            //     // Eigen::MatrixXf mesh_fV(4,3);
            //     // Eigen::Matrix3f fVmat3 = mesh->getFlatV();
            //     // mesh_fV << 
            //     //     fVmat3.row(0),
            //     //     fVmat3.row(1),
            //     //     fVmat3.row(2),
            //     //     1, 1, 1;
            // for (int i = 0; i < this->fV.cols(); i += 3) {
            //     Eigen::MatrixXf mesh_fV(4,3);
            //     mesh_fV << 
            //         this->fV.col(0),
            //         this->fV.col(1),
            //         this->fV.col(2),
            //         1, 1, 1;
            //     mesh_fV = this->ModelMat*mesh_fV;

            //     if (mesh->getIntersection(ray_origin, ray_direction, intersection, mesh_fV)) {
            //         intersected = true;
            //         if (intersection == ray_origin) dist = 0;
            //         else dist = std::fmin((intersection-ray_origin).norm(), dist);
            //     }
            // }
            return intersected;
        }
        void translate(Eigen::Vector4f delta) {
            Eigen::MatrixXf T = Eigen::MatrixXf::Identity(4, 4);
            T.col(3)(0) = delta(0); T.col(3)(1) = delta(1); T.col(3)(2) = delta(2);
            this->update_Model_Mat(T, true);
        }
        void scale(double factor) {
            // double factor = 1+delta;
            Eigen::MatrixXf S = Eigen::MatrixXf::Identity(4, 4);
            S.col(0)(0) = factor; S.col(1)(1) = factor; S.col(2)(2) = 1.;
            Eigen::MatrixXf I = Eigen::MatrixXf::Identity(4,4);
            S = (2*I-this->T_to_ori)*S*(this->T_to_ori);
            this->update_Model_Mat(S, false);
        }
        void rotate(double degree, Eigen::Matrix4f rotateMat) {
            double r = degree*PI/180.0;
            Eigen::MatrixXf R = Eigen::MatrixXf::Identity(4, 4);
            R.col(0)(0) = std::cos(r); R.col(0)(1) = std::sin(r);
            R.col(1)(0) = -std::sin(r); R.col(1)(1) = std::cos(r);
            Eigen::MatrixXf I = Eigen::MatrixXf::Identity(4,4);
            R = (2*I-this->T_to_ori)*rotateMat*(this->T_to_ori);
            this->update_Model_Mat(R, false);
        }
        void update_Model_Mat(Eigen::MatrixXf M, bool left) {
            if (left) {
                // left cross mul
                this->ModelMat = M*this->ModelMat;
            }
            else {
                // right cross mul
                this->ModelMat = this->ModelMat*M;
            }
        }
        std::string export_svg(Camera *camera) {
            std::stringstream ss;
            for (int i = 0; i < this->fV.cols(); i += 3) {
                Eigen::MatrixXf mesh_fV = this->fV.block<3,3>(0,i);
                mesh_fV = mat_to_4(mesh_fV);
                mesh_fV = camera->get_project_mat()*camera->flatViewMat*ModelMat*mesh_fV;

                std::string svg_str = get_tri_g_template();
                std::string ax = std::to_string(mesh_fV.col(0).x()), ay = std::to_string(mesh_fV.col(0).y());
                std::string bx = std::to_string(mesh_fV.col(1).x()), by = std::to_string(mesh_fV.col(1).y());
                std::string cx = std::to_string(mesh_fV.col(2).x()), cy = std::to_string(mesh_fV.col(2).y());
                svg_str = replace_all(svg_str, "$AX", ax); svg_str = replace_all(svg_str, "$AY", ay);
                svg_str = replace_all(svg_str, "$BX", bx); svg_str = replace_all(svg_str, "$BY", by);
                svg_str = replace_all(svg_str, "$CX", cx); svg_str = replace_all(svg_str, "$CY", cy);

                ss << svg_str << std::endl;
            }
            std::string svg_str = ss.str();
            return svg_str;
        }
        void adjustSize(Eigen::MatrixXf boundingBox) {
            double maxx = boundingBox.col(1)(0), maxy = boundingBox.col(1)(1);
            double minx = boundingBox.col(0)(0), miny = boundingBox.col(0)(1);
            Eigen::MatrixXf curBox = get_bounding_box_2d(this->fV);
            this->barycenter = (curBox.col(0) + curBox.col(1))/2.0;
            this->barycenter(2) = 0; this->barycenter(3) = 1;

            // Computer the translate Matrix from barycenter to the origin
            Eigen::Vector4f delta = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0) - this->barycenter;
            T_to_ori.col(3)(0) = delta(0); T_to_ori.col(3)(1) = delta(1); T_to_ori.col(3)(2) = delta(2);

            // adjust inital position according to bounding box
            double scale_factor = fmin(1.0/(maxx-minx), 1.0/(maxy-miny));
            this->translate(delta);
            this->scale(scale_factor);
        }
};
class _3dObject {
    public:
        Eigen::MatrixXf box;
        Eigen::MatrixXf ModelMat;
        Eigen::MatrixXf ModelMat_T;
        Eigen::MatrixXf Adjust_Mat;
        Eigen::MatrixXf T_to_ori;
        Eigen::Vector4f barycenter;
        Eigen::VectorXi IDX;
        Eigen::MatrixXf V;
        Eigen::MatrixXf C;
        Eigen::MatrixXf Normals;

        int render_mode;
        double r, s, tx, ty;
        BezierCurve* bc;
        // FlattenObject* flattenObj;
        std::vector<FlattenObject> flattenObjs;
        std::set<int> selectedMeshes;

        std::vector<std::vector<Mesh*>> edges;
        std::vector<Mesh*> meshes;

        VertexArrayObject VAO;
        VertexBufferObject VBO_P;
        VertexBufferObject VBO_C;
        VertexBufferObject VBO_N;
        IndexBufferObject IBO_IDX;

        _3dObject(){}
        _3dObject(std::string off_path, int color_idx) {
            //load from off file
            Eigen::MatrixXf V, C;
            Eigen::VectorXi IDX;
            loadMeshfromOFF(off_path, V, C, IDX);
            C = Eigen::MatrixXf(3, V.cols());
            // int color_idx = rand() % colors.size();
            Eigen::Vector3i color = colors[color_idx];
            for (int i = 0; i < V.cols(); i++) {
                C.col(i) = color.cast<float>();
            }
            //compute the bouncing box
            box = get_bounding_box(V);
            //create class Mesh for each mech
            this->initial(V, C, IDX, box);
        }
        void initial(Eigen::MatrixXf V, Eigen::MatrixXf C, Eigen::VectorXi IDX, Eigen::MatrixXf bounding_box) {
            // make sure it is a point
            for (int i = 0; i < V.cols(); i++) {
                V.col(i)(3) = 1.0;
            }
            this->IDX = IDX;
            this->V = V;
            this->C = C;

            // Create a VAO
            this->VAO.init();
            this->VAO.bind();
            // Initialize the VBO with the vertices data
            this->VBO_P.init();
            this->VBO_P.update(V);
            this->VBO_C.init();
            this->VBO_C.update(C/255.0);
            this->VBO_N.init();
            this->IBO_IDX.init();
            this->IBO_IDX.update(IDX);

            this->ModelMat = Eigen::MatrixXf::Identity(4,4);
            this->ModelMat_T = Eigen::MatrixXf::Identity(4,4);
            this->T_to_ori = Eigen::MatrixXf::Identity(4,4);
            this->r = 0; this->s = 1; this->tx = 0; this->ty = 0;
            this->render_mode = 0;
            this->barycenter = (bounding_box.col(0) + bounding_box.col(1))/2.0;

            // Computer the translate Matrix from barycenter to the origin
            Eigen::Vector4f delta = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0) - this->barycenter;
            T_to_ori.col(3)(0) = delta(0); T_to_ori.col(3)(1) = delta(1); T_to_ori.col(3)(2) = delta(2);

            // Adjust size
            this->initial_adjust(bounding_box);

            // initial edges
            for (int i = 0; i < V.cols(); i++) {
                this->edges.push_back(std::vector<Mesh*>());
            }
            std::cout << "start generating Meshs" << std::endl;
            for (int i = 0; i < IDX.rows(); i+=3) {
                int a = IDX(i), b = IDX(i+1), c = IDX(i+2);
                Eigen::MatrixXf Vblock(4, 3);
                Vblock << V.col(a), V.col(b), V.col(c);
                auto mesh = new Mesh(Vblock, bounding_box);
                this->meshes.push_back(mesh);
                this->edges[a].push_back(mesh);
                this->edges[b].push_back(mesh);
                this->edges[c].push_back(mesh);
            }

            // Compute normlas for each vertex
            if (this->edges.size() != V.cols()) {
                std::cout << "Assert failed" << std::endl;
                exit(1);
            }
            this->Normals.resize(V.rows(), V.cols());
            for (int i = 0; i < V.cols(); i++) {
                Eigen::Vector4f normal(0., 0., 0., 0.);
                for (Mesh* mesh: this->edges[i]) {
                    normal += mesh->normal;
                }
                this->Normals.col(i) = normal.normalized();
            }
            this->VBO_N.update(this->Normals);

            bc = new BezierCurve();

            std::cout << "meshes # = " << this->meshes.size() << std::endl;
            std::cout << "finish" << std::endl;

            // create flatten object
            // this->flattenObj = nullptr;
            // this->flattenObjs.resize(10);
            this->flatten();
        }
        void initial_adjust(Eigen::MatrixXf bounding_box) {
            double maxx = bounding_box.col(1)(0), maxy = bounding_box.col(1)(1), maxz = bounding_box.col(1)(2);
            double minx = bounding_box.col(0)(0), miny = bounding_box.col(0)(1), minz = bounding_box.col(0)(2);
            double scale_factor = fmin(1.0/(maxx-minx), fmin(1.0/(maxy-miny), 1.0/(maxz-minz)));
            this->scale(scale_factor);
        }
        bool hit(Eigen::Vector4f ray_origin, Eigen::Vector4f ray_direction, float &dist) {

            bool intersected = false;
            int cnt = 0;
            int selectedMeshId;
            for (auto mesh : this->meshes) {
                Eigen::Vector4f intersection;
                Eigen::MatrixXf mesh_V(4,3);
                int i = this->IDX(cnt*3), j = this->IDX(cnt*3+1), k = this->IDX(cnt*3+2);
                mesh_V << this->V.col(i), this->V.col(j), this->V.col(k);
                mesh_V = this->ModelMat*mesh_V;

                // the ray is parallel to the Mesh, no solution
                Eigen::Vector4f world_normal = ((this->ModelMat.inverse()).transpose()*mesh->normal).normalized();
                if (ray_direction.dot(world_normal) == 0) {
                    cnt++;
                    continue;
                }

                if (mesh->getIntersection(ray_origin, ray_direction, intersection, mesh_V)) {
                    intersected = true;
                    double curDist = (intersection-ray_origin).norm();
                    if (curDist < dist) {
                        selectedMeshId = cnt;
                        dist = curDist;
                    }
                }
                cnt++;
            }
            if (intersected) {
                if (this->selectedMeshes.find(selectedMeshId) != selectedMeshes.end())
                    this->selectedMeshes.erase(selectedMeshId);
                else
                    this->selectedMeshes.insert(selectedMeshId);
            }
            return intersected;
        }
        void translate(Eigen::Vector4f delta) {
            // delta = this->Adjust_Mat.inverse()*delta;
            this->tx += delta(0); this->ty += delta(1);
            Eigen::MatrixXf T = Eigen::MatrixXf::Identity(4, 4);
            T.col(3)(0) = delta(0); T.col(3)(1) = delta(1); T.col(3)(2) = delta(2);
            Eigen::MatrixXf T_T = Eigen::MatrixXf::Identity(4, 4);
            T_T.col(3)(0) = -delta(0); T_T.col(3)(1) = -delta(1); T_T.col(3)(2) = -delta(2);
            this->update_Model_Mat(T, T_T, true);
        }
        void scale(double factor) {
            // double factor = 1+delta;
            this->s *= factor;
            Eigen::MatrixXf S = Eigen::MatrixXf::Identity(4, 4);
            S.col(0)(0) = factor; S.col(1)(1) = factor; S.col(2)(2) = factor;
            Eigen::MatrixXf S_T = Eigen::MatrixXf::Identity(4, 4);
            S_T.col(0)(0) = 1.0/factor; S_T.col(1)(1) = 1.0/factor; S_T.col(2)(2) = 1.0/factor;

            Eigen::MatrixXf I = Eigen::MatrixXf::Identity(4,4);
            S = (2*I-this->T_to_ori)*S*(this->T_to_ori);
            S_T = (2*I-this->T_to_ori)*S_T*(this->T_to_ori);

            this->update_Model_Mat(S, S_T, false);
        }
        void rotate(double degree, Eigen::Matrix4f rotateMat) {
            double r = degree*PI/180.0;
            this->r += r;
            Eigen::MatrixXf R = Eigen::MatrixXf::Identity(4, 4);
            R.col(0)(0) = std::cos(r); R.col(0)(1) = std::sin(r);
            R.col(1)(0) = -std::sin(r); R.col(1)(1) = std::cos(r);
            Eigen::MatrixXf R_T = Eigen::MatrixXf::Identity(4, 4);
            R_T.col(0)(0) = std::cos(r); R_T.col(0)(1) = -std::sin(r);
            R_T.col(1)(0) = std::sin(r); R_T.col(1)(1) = std::cos(r);

            Eigen::MatrixXf I = Eigen::MatrixXf::Identity(4,4);
            R = (2*I-this->T_to_ori)*rotateMat*(this->T_to_ori);
            R_T = (2*I-this->T_to_ori)*rotateMat.inverse()*(this->T_to_ori);

            this->update_Model_Mat(R, R_T, false);
        }
        void update_Model_Mat(Eigen::MatrixXf M, Eigen::MatrixXf M_T, bool left) {
            if (left) {
                // left cross mul
                this->ModelMat = M*this->ModelMat;
                this->ModelMat_T = this->ModelMat_T*M_T;
            }
            else {
                // right cross mul
                this->ModelMat = this->ModelMat*M;
                this->ModelMat_T = M_T*this->ModelMat_T;
            }
        }
        void flatten() {
            // delete all flatten object first
            this->flattenObjs.clear();

            // create a new flatten object using selected meshes
            // use all meshes if no mesh is selected
            Eigen::VectorXi selectedIDX;
            if (this->selectedMeshes.size() == 0) {
                selectedIDX = this->IDX;
            }
            else {
                selectedIDX.resize((selectedMeshes.size()*3));
                int i = 0;
                for (auto meshId: this->selectedMeshes) {
                    selectedIDX(i++) = this->IDX(meshId*3);
                    selectedIDX(i++) = this->IDX(meshId*3+1);
                    selectedIDX(i++) = this->IDX(meshId*3+2);
                }
            }
            std::cout << "starts flattening" << std::endl;
            std::vector<bool> meshFlattened(selectedIDX.rows()/3, false);

            int flattedCnt = 0;
            while (flattedCnt < selectedIDX.rows()/3) {
                this->flattenObjs.push_back(FlattenObject(this->V, selectedIDX, meshFlattened));
                flattedCnt = 0;
                for (int i = 0; i < meshFlattened.size(); i++) {
                    flattedCnt += meshFlattened[i];
                }
            }

            // scale all islands with a same ratio to fit the window
            std::vector<Eigen::Matrix2f> islandsBoxs;
            Eigen::MatrixXf boundingBox(2, 2);
            double deltaY = 0.;
            for (FlattenObject &flatObj: this->flattenObjs) {
                Eigen::MatrixXf box = get_bounding_box_2d(flatObj.fV);
                islandsBoxs.push_back(box);
                if (box.col(1)(1)-box.col(0)(1) > deltaY) {
                    deltaY = box.col(1)(1)-box.col(0)(1);
                    boundingBox = box;
                }
            }
            std::cout << "island # = " << this->flattenObjs.size() << std::endl;
            for (FlattenObject &flatObj: this->flattenObjs) {
                std::cout << "mesh # = " << flatObj.fV.cols()/3 << std::endl;
                // flatObj.adjustSize(boundingBox);
            }

            // arrange the layout of islands on paper
            double paperL = 0., paperT = 0., paperR = 0., paperB = 0.;
            double curX = paperL, curY = paperT;
            double margin = 0.1;
            for (int i = 0; i < this->flattenObjs.size(); i++) {
                FlattenObject &flatObj = this->flattenObjs[i];
                Eigen::Matrix2f box = islandsBoxs[i];
                double w = box.col(1).x()-box.col(0).x(), h = box.col(1).y()-box.col(0).y();
                islandMoveTo(paperL, curY, box, flatObj);
                curY -= h+margin;
                paperR = fmax(paperR, w);
                paperB = fmin(paperB, curY);
            }

            // scale the whole paper to fit the window
            double scaleFactor = fmin(1.0/(paperT-paperB), 1.0/(paperR-paperL));
            Eigen::MatrixXf S = Eigen::MatrixXf::Identity(4, 4);
            S.col(0)(0) = scaleFactor; S.col(1)(1) = scaleFactor;
            // std::cout << "scaleFactor" << std::endl;
            // std::cout << scaleFactor << std::endl;
            for (FlattenObject &flatObj: this->flattenObjs) {
                flatObj.ModelMat = S*flatObj.ModelMat;
            }

            // move paper center to the center of the screen
            // std::cout << "paperL, paperR, paperT, paperBottom" << std::endl;
            // std::cout << paperL << " " << paperR << " " << paperT << " " << paperB << std::endl;
            Eigen::Vector4f paperCenter(scaleFactor*(paperL+paperR)/2.0, scaleFactor*(paperT+paperB)/2.0, 0., 1.);
            // std::cout << "paperCenter" << std::endl;
            // std::cout << paperCenter << std::endl;
            Eigen::Vector4f delta = Eigen::Vector4f(0., 0., 0., 1.)-paperCenter;
            for (FlattenObject &flatObj: this->flattenObjs) {
                flatObj.translate(delta);
            }
        }
        void islandMoveTo(double l, double t, Eigen::Matrix2f boundBox, FlattenObject &flatObj) {
            Eigen::Vector2f leftTop = Eigen::Vector2f(l, t);
            double bminx = boundBox.col(0).x(), bmaxy = boundBox.col(1).y();
            Eigen::Vector4f bleftTop = Eigen::Vector4f(bminx, bmaxy, 0., 1.);
            bleftTop = flatObj.ModelMat*bleftTop;
            Eigen::Vector2f delta = leftTop - Eigen::Vector2f(bleftTop.x(), bleftTop.y());
            std::cout << "island move delta" << std::endl;
            std::cout << delta << std::endl;
            flatObj.translate(Eigen::Vector4f(delta(0), delta(1), 0., 0.));
        }
};
class Cube: public _3dObject {
    public:
        Eigen::Vector4f center;
        Cube(int color_idx, double x, double y, double z, double len = 1.0):_3dObject() {
            std::cout << "enter initialization" << std::endl;
            this->center = Eigen::Vector4f(x, y, z, 1);
            Eigen::Vector4f HB = Eigen::Vector4f(-len, len, len, 0);
            Eigen::Vector4f EC = Eigen::Vector4f(len, len, len, 0);
            Eigen::Vector4f GA = Eigen::Vector4f(-len, len, -len, 0);
            Eigen::Vector4f FD = Eigen::Vector4f(len, len, -len, 0);
            Eigen::Vector4f A = center+0.5*GA, G = center-0.5*GA;
            Eigen::Vector4f B = center+0.5*HB, H = center-0.5*HB;
            Eigen::Vector4f C = center+0.5*EC, E = center-0.5*EC;
            Eigen::Vector4f D = center+0.5*FD, F = center-0.5*FD;
            // 6 faces, 12 Meshs
            Eigen::MatrixXf V(4, 8);
            V << A, B, C, D, E, F, G, H;
            Eigen::VectorXi IDX(36);
            IDX << 0,1,2, 2,3,0, 4,6,5, 6,4,7, 0,5,1, 5,0,4, 1,6,2, 1,5,6, 3,2,6, 6,7,3, 3,7,0, 7,4,0;
            Eigen::MatrixXf Color(3, 8);
            Eigen::Vector3i color = colors[color_idx];
            for (int i = 0; i < 8; i++) {
                Color.col(i) = color.cast<float>();
            }
            std::cout << "ready to generate Meshs" << std::endl;
            // bounding box
            Eigen::MatrixXf box(4, 2);
            box << x-len/2.0, x+len/2.0, y-len/2.0, y+len/2.0, z-len/2.0, z+len/2.0, 1, 1;
            this->initial(V, Color, IDX, box);
        }
};
class _3dObjectBuffer {
    public:
        std::vector<_3dObject*> _3d_objs;
        _3dObject* selected_obj;
        FlattenObject* selected_flat_obj;
        RayTracer* ray_tracer;
        int color_idx;

        _3dObjectBuffer() {
            selected_obj = nullptr;
            selected_flat_obj = nullptr;
            ray_tracer = new RayTracer();
            ray_tracer->add_light(Eigen::Vector4f(0.0, 0.0, 1, 1.0));
            color_idx = 0;
        }
        void add_cube(double x, double y, double z, double len=1.0) {
            this->color_idx = (this->color_idx)%colors.size();
            this->color_idx++;
            _3d_objs.push_back(new Cube(this->color_idx, x, y, z, len));
        }
        void add_object(std::string off_path) {
            this->color_idx = (this->color_idx)%colors.size();
            this->color_idx++;
            _3d_objs.push_back(new _3dObject(off_path, this->color_idx));
        }
        bool hit(int subWindow, Eigen::Vector4f ray_origin, Eigen::Vector4f ray_direction, float &ret_dist, int mode=CILCK_ACTION) {
            float min_dist = DIST_MAX;
            _3dObject* selected = nullptr;
            FlattenObject* selected_flat = nullptr;
            if (subWindow == LEFTSUBWINDOW) {
                // find the hitted 3D object if any
                for (auto obj: _3d_objs) {
                    float dist = DIST_MAX;
                    if (obj->hit(ray_origin, ray_direction, dist)) {
                        if (selected == nullptr || min_dist > dist) {
                            min_dist = dist;
                            selected = obj;
                        }
                    }
                }
            }
            else if (subWindow == RIGHTSUBWINDOW) {
                // find the hitted flat object if any
                for (auto obj: _3d_objs) {
                    float dist = DIST_MAX;
                    for (auto &flatObj: obj->flattenObjs) {
                        if (flatObj.hit(ray_origin, ray_direction, dist)) {
                            if (selected_flat == nullptr || min_dist > dist) {
                                min_dist = dist;
                                selected_flat = &flatObj;
                            }
                        }
                    }
                }
            }
            ret_dist = min_dist;

            if (mode == CILCK_ACTION) {
                this->selected_obj = selected;
                this->selected_flat_obj = selected_flat;
            }

            return selected != nullptr || selected_flat != nullptr;
        }
        bool translate(Eigen::Vector4f delta) {
            if (this->selected_obj != nullptr) {
                this->selected_obj->translate(delta);
                return true;
            }
            else if (this->selected_flat_obj != nullptr) {
                this->selected_flat_obj->translate(delta);
                return true;
            }
            return false;
        }
        bool rotate(double degree, Eigen::Vector3f rotateAixs) {
            if (this->selected_obj != nullptr) {
                auto M = this->get_rotate_mat(degree, rotateAixs);
                this->selected_obj->rotate(degree, M);
                return true;
            }
            else if (this->selected_flat_obj != nullptr) {
                auto M = this->get_rotate_mat(degree, rotateAixs);
                this->selected_flat_obj->rotate(degree, M);
                return true;
            }
            return false;
        }
        bool scale(double factor) {
            if (this->selected_obj != nullptr) {
                this->selected_obj->scale(factor);
                return true;
            }
            else if (this->selected_flat_obj != nullptr) {
                this->selected_flat_obj->scale(factor);
                return true;
            }
            return false;
        }
        bool switch_render_mode(int mode) {
            if (this->selected_obj == nullptr) return false;
            this->selected_obj->render_mode = mode;
            return true;
        }
        bool delete_obj() {
            if (this->selected_obj != nullptr) {
                this->_3d_objs.erase(std::remove(this->_3d_objs.begin(), this->_3d_objs.end(), this->selected_obj), this->_3d_objs.end());
                delete this->selected_obj;
                this->selected_obj = nullptr;
                return true;
            }
            return false;
        }
        Eigen::Matrix4f get_rotate_mat(float angle, Eigen::Vector3f rotateAixs) {
            float r = (PI/180) * (angle/2);
            float x = rotateAixs.x() * sin(r);
            float y = rotateAixs.y() * sin(r);
            float z = rotateAixs.z() * sin(r);
            float w = cos(r);
            Eigen::Matrix4f rotate_mat;
            rotate_mat << 
            1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w, 0,
            2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*x-2*x*w, 0,
            2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y, 0,
            0,0,0,1;
            return rotate_mat;
        }
        std::string export_flat_svg(Camera *camera) {
            std::stringstream ss;
            for (auto it = _3d_objs.rbegin(); it != _3d_objs.rend(); it++) {
                for (FlattenObject &flatObj: (*it)->flattenObjs) {
                    ss << flatObj.export_svg(camera) << std::endl;
                }
            }
            std::string svg_str = ss.str();
            return svg_str;
        }
};

_3dObjectBuffer* _3d_objs_buffer;
std::vector<_3dObject*>* _3d_objs;
Camera *camera;
std::chrono::high_resolution_clock::time_point t_pre;
bool is_playing = false;
int playing_cnt = 0;
double unitTime = 0.;
Eigen::MatrixXf AnimeT;

void export_svg(GLFWwindow* window) {
    std::string svg_str = get_svg_root_template();
    // adjust aspect ratio
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    double aspect = (double)width/(double)height;
    svg_str = replace_all(svg_str, "$d", std::to_string(-1.0/aspect));
    std::cout << "w: " << width << "h: " << height << std::endl;
    std::cout << "aspect " << aspect << std::endl;
    // flatten objects
    std::string flat_svg_str = _3d_objs_buffer->export_flat_svg(camera);
    svg_str = replace_all(svg_str, "$TG", flat_svg_str);
    // Save svg to file
    std::ofstream svg_file;
    svg_file.open ("export.svg");
    svg_file << svg_str;
    svg_file.close();
}

void update_window_scale(GLFWwindow* window) {
    camera->update_project_mat(window);
    camera->look_at(window);
}

Eigen::Vector4f get_click_position(GLFWwindow* window, int &subWindow) {
    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    Eigen::Vector4f p_screen;
    // click on right screen
    if (xpos > width/2) {
        subWindow = RIGHTSUBWINDOW;
        p_screen = Eigen::Vector4f((xpos-width/2)*2,height-1-ypos,0.0,1.0);
    }
    // click on left screen
    else {
        subWindow = LEFTSUBWINDOW;
        p_screen = Eigen::Vector4f(xpos*2,height-1-ypos,0.0,1.0);
    }
    Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,-camera->n,1.0);
    Eigen::Vector4f p_camera = camera->get_project_mat().inverse() * p_canonical;
    if (fabs(p_camera(3)-1.0) > 0.001) {
        p_camera = p_camera/p_camera(3);
    }

    Eigen::Vector4f p_world;
    if (subWindow == RIGHTSUBWINDOW)
        p_world = camera->flatViewMat.inverse() * p_camera;
    else if (subWindow == LEFTSUBWINDOW)
        p_world = camera->ViewMat.inverse() * p_camera;

    return p_world;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Convert screen position to world coordinates
    int subWindow;
    Eigen::Vector4f click_point = get_click_position(window, subWindow);

    // Update the position of the first vertex if the left button is pressed
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        Eigen::Vector4f ray_origin = click_point;
        // orth projection
        Eigen::Vector4f ray_direction;
        if (subWindow == RIGHTSUBWINDOW) {
            ray_direction = Eigen::Vector4f(0., 0., -1., 0.);
        }
        else {
            if (camera->project_mode == ORTHO) {
                ray_direction = -to_4_vec(camera->forward);
            }
            else if (camera->project_mode == PERSPECT) {
                ray_direction = (click_point-to_4_vec(camera->position)).normalized();
                ray_direction(3) = 0.0;
            }
        }
        float dist = 0;
        if (_3d_objs_buffer->hit(subWindow, ray_origin, ray_direction, dist)) {
            drag = true;
            hit_dist = dist;
            pre_cursor_point = click_point;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        drag = false;
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    // Convert screen position to world coordinates
    int subWindow;
    Eigen::Vector4f click_point = get_click_position(window, subWindow);

    if (drag) {
        Eigen::Vector4f delta = click_point-pre_cursor_point;
        _3d_objs_buffer->translate(delta);
        pre_cursor_point = click_point;
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key)
    {
        // export svg
        case  GLFW_KEY_0:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "export SVG");
                export_svg(window);
            }
            break;
        // Add a cube
        case  GLFW_KEY_1:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a unit cube");
                _3d_objs_buffer->add_cube(0, 0, 0);
            }
            break;
        // Add a Cube from OFF
        case  GLFW_KEY_2:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a cube");
                _3d_objs_buffer->add_object(CUBE_OFF_PATH);
            }
            break;
        // Add a Cone
        case  GLFW_KEY_3:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a cone");
                _3d_objs_buffer->add_object(CONE_OFF_PATH);
            }
            break;
        // Add a Ball
        case  GLFW_KEY_4:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a cone");
                _3d_objs_buffer->add_object(BALL_OFF_PATH);
            }
            break;
        // Add a Fox
        case  GLFW_KEY_5:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a fox");
                _3d_objs_buffer->add_object(FOX_OFF_PATH);
            }
            break;
        // Add a Bunny
        case  GLFW_KEY_6:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "add a bunny");
                _3d_objs_buffer->add_object(BUNNY_OFF_PATH);
            }
            break;
         // Deleta an object
        case  GLFW_KEY_DELETE:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->delete_obj()) {
                    glfwSetWindowTitle (window, "deleted one object");
                }
            }
            break;
        // Clockwise rotate 10 degree
        case  GLFW_KEY_H:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->rotate(10, camera->forward)) {
                    glfwSetWindowTitle (window, "clockwise rotate 10");
                }
            }
            break;
        // Counter-clockwise rotate 10 degree
        case  GLFW_KEY_J:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->rotate(-10, camera->forward)) {
                    glfwSetWindowTitle (window, "counter-clockwise rotate 10");
                }
            }
            break;
        // Scale up 25%
        case GLFW_KEY_K:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->scale(1.25)) {
                    glfwSetWindowTitle (window, "scale up 25%");
                }
            }
            break;
        // Scale down 25%
        case GLFW_KEY_L:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->scale(1.0/1.25)) {
                    glfwSetWindowTitle (window, "scale down 25%");
                }
            }
            break;
        // move object up (camera up direction)
        case GLFW_KEY_W:
            if (action == GLFW_PRESS) {
                // if (_3d_objs_buffer->translate(Eigen::Vector4f(0., 0.2, 0., 0.))) {
                if (_3d_objs_buffer->translate(to_4_vec(camera->up)/10.0)) {
                    glfwSetWindowTitle (window, "move up");
                }
            }
            break;
        // move object down (camera -up direction)
        case GLFW_KEY_S:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->translate(-to_4_vec(camera->up)/10.0)) {
                    glfwSetWindowTitle (window, "move down");
                }
            }
            break;
        // move object left (camera -right direction)
        case GLFW_KEY_A:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->translate(-to_4_vec(camera->right)/10.0)) {
                    glfwSetWindowTitle (window, "move left");
                }
            }
            break;
        // move object right (camera right direction)
        case GLFW_KEY_D:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->translate(to_4_vec(camera->right)/10.0)) {
                    glfwSetWindowTitle (window, "move right");
                }
            }
            break;
        // move object toward camera (camera -forward direction)
        case GLFW_KEY_E:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->translate(-to_4_vec(camera->forward)/10.0)) {
                    glfwSetWindowTitle (window, "move toward camera");
                }
            }
            break;
        // move object away from camera (camera forward direction)
        case GLFW_KEY_Q:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->translate(to_4_vec(camera->forward)/10.0)) {
                    glfwSetWindowTitle (window, "move away from camera");
                }
            }
            break;
        // Move the camera
        case GLFW_KEY_LEFT:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "rotate camera to left 10 degree");
                camera->rotate(0, -10);
                camera->look_at(window);
            }
            break;
        case GLFW_KEY_RIGHT:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "rotate camera to right 10 degree");
                camera->rotate(0, 10);
                camera->look_at(window);
            }
            break;
        case GLFW_KEY_UP:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "rotate camera to up 10 degree");
                camera->rotate(10, 0);
                camera->look_at(window);
            }
            break;
        case GLFW_KEY_DOWN:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "rotate camera to down 10 degree");
                camera->rotate(-10, 0);
                camera->look_at(window);
            }
            break;
        // zoom in / zoom out
        case GLFW_KEY_EQUAL:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "zoom in 10%");
                camera->zoom(0.9);
                camera->look_at(window);
            }
            break;
        case GLFW_KEY_MINUS:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "zoom out 10%");
                camera->zoom(1.0/0.9);
                camera->look_at(window);
            }
            break;
        // reset camera
        case GLFW_KEY_R:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "reset camera");
                camera->reset();
                camera->look_at(window);
            }
            break;
        // switch project mode
        case GLFW_KEY_M:
            if (action == GLFW_PRESS) {
                camera->switch_project_mode();
                if (camera->project_mode == ORTHO)
                    glfwSetWindowTitle (window, "switch to ortho projection");
                else
                    glfwSetWindowTitle (window, "switch to perspective projection");
            }
            break;
        // flatten the selected 3d object
        case GLFW_KEY_F:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->selected_obj != nullptr) {
                    glfwSetWindowTitle (window, "flatten the selected 3d object");
                    _3d_objs_buffer->selected_obj->flatten();
                    _3d_objs_buffer->selected_flat_obj = nullptr;
                    _3d_objs_buffer->selected_obj = nullptr;
                }
            }
            break;
        default:
            break;
    }
}

int main(void)
{
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "3D Editor", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec4 position;"
                    "uniform vec3 color;"
                    "uniform mat4 ViewMat;"
                    "uniform mat4 ModelMat;"
                    "uniform mat4 ProjectMat;"
                    "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    gl_Position = ProjectMat*ViewMat*ModelMat*position;"
                    "    f_color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 f_color;"
                    "out vec4 outColor;"
                    "uniform vec3 lineColor;"
                    "uniform int isLine;"
                    "void main()"
                    "{"
                    "    if (isLine == 1)"
                    "       outColor = vec4(lineColor, 1.0);"
                    "    else"
                    "       outColor = vec4(f_color, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Register the mouse cursor position callback
    glfwSetCursorPosCallback(window, cursor_position_callback);

    // Update viewport
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Inital control objects
    _3d_objs_buffer = new _3dObjectBuffer();
    camera = new Camera(window);

    // bind light
    auto light = _3d_objs_buffer->ray_tracer->lights[0];
    glUniform4fv(program.uniform("light.position"), 1, light->position.data());
    glUniform3fv(program.uniform("light.intensities"), 1, light->intensity.data());
    // bind special colors
    Eigen::Vector3f special_color = (SELCET_COLOR.cast<float>())/255.0;
    // glUniform3fv(program.uniform("selectedColor"), 1, special_color.data());
    special_color = (BLACK.cast<float>())/255.0;
    // special_color = (WHITE.cast<float>())/255.0;
    glUniform3fv(program.uniform("lineColor"), 1, special_color.data());

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        // Bind your program
        program.bind();

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Enable depth test
        glEnable(GL_DEPTH_TEST);

        glEnable(GL_PROGRAM_POINT_SIZE);

        // Update window scaling
        update_window_scale(window);
        // Send the View Mat to Vertex Shader
        Eigen::Vector4f tmp;
        tmp(0) = camera->position(0); tmp(1) = camera->position(1); tmp(2) = camera->position(2);
        tmp(3) = 1.0;
        Eigen::Vector3f white = (WHITE.cast<float>())/255.0;
        // glUniform3fv(program.uniform("color"), 1, special_color.data());
        Eigen::Vector3f red = (RED.cast<float>())/255.0;
        glUniform4fv(program.uniform("viewPosition"), 1, tmp.data());
        glUniformMatrix4fv(program.uniform("ViewMat"), 1, GL_FALSE, camera->flatViewMat.data());
        glUniformMatrix4fv(program.uniform("ProjectMat"), 1, GL_FALSE, camera->get_project_mat().data());

        int WindowWidth, WindowHeight;
        glfwGetWindowSize(window, &WindowWidth, &WindowHeight);

        // right screen
        glViewport(WindowWidth, 0, WindowWidth, WindowHeight*2);

        for (auto obj: _3d_objs_buffer->_3d_objs) {
            // prepare
            for (FlattenObject &flatObj: obj->flattenObjs) {
                // auto flatObj = obj->flattenObj;
                glUniformMatrix4fv(program.uniform("ModelMat"), 1, GL_FALSE, flatObj.ModelMat.data());

                flatObj.VAO.bind();
                program.bindVertexAttribArray("position", flatObj.VBO_P);
                glUniform3fv(program.uniform("color"), 1, white.data());
                for (int i = 0; i < flatObj.fV.cols(); i += 3) {
                    glUniform1i(program.uniform("isLine"), 1);
                    glDrawArrays(GL_LINE_LOOP, i, 3);
                    glUniform1i(program.uniform("isLine"), 0);
                    glDrawArrays(GL_TRIANGLES, i, 3);
                }
            }
        }

        // left screen
        glViewport(0, 0, WindowWidth, WindowHeight*2);
        glUniformMatrix4fv(program.uniform("ViewMat"), 1, GL_FALSE, camera->ViewMat.data());

        for (auto obj: _3d_objs_buffer->_3d_objs) {
            obj->VAO.bind();
            program.bindVertexAttribArray("position",obj->VBO_P);
            glUniformMatrix4fv(program.uniform("ModelMat"), 1, GL_FALSE, obj->ModelMat.data());
            for (int i = 0; i < obj->meshes.size(); i++) {
                if (obj->selectedMeshes.find(i) != obj->selectedMeshes.end()) {
                    glUniform3fv(program.uniform("color"), 1, red.data());
                }
                else {
                    glUniform3fv(program.uniform("color"), 1, white.data());
                }
                glUniform1i(program.uniform("isLine"), 1);
                glDrawElements(GL_LINE_LOOP, 3, GL_UNSIGNED_INT, (void*)(sizeof(int)* (i*3)));
                glUniform1i(program.uniform("isLine"), 0);
                glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, (void*)(sizeof(int)* (i*3)));
            }
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        if (playing_cnt > 0) continue;
        
        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}