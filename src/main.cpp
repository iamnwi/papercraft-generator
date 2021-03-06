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
#include <iomanip>

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

// Timer
#include <chrono>

typedef std::pair<int, int> Edge;

// Contains the vertex positions
Eigen::MatrixXd V(2,3);

// Contains the per-vertex color
Eigen::MatrixXd C(3,3);

// The view matrix
// Eigen::MatrixXd ViewMat(4,4);
Eigen::MatrixXd ProjectMat(4,4);

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
double pre_aspect_ratio = -1;
bool pre_aspect_is_x = false;
bool drag = false;
double hit_dist;
Eigen::Vector4d pre_cursor_point;

class Camera {
    public:
        Eigen::Matrix4d flatViewMat;
        Eigen::Matrix4d ViewMat;
        Eigen::Matrix4d ortho_mat;
        Eigen::Matrix4d perspect_mat;
        Eigen::Vector3d position;
        Eigen::Vector3d global_up;
        Eigen::Vector3d up;
        Eigen::Vector3d right;
        Eigen::Vector3d forward;
        Eigen::Vector3d target;

        // bounding box
        double n, f, t, b, r, l;
        int project_mode;
        double theta;
        int phi, zeta;
        double radius;

        Camera(GLFWwindow* window, Eigen::Vector3d position=Eigen::Vector3d(0.0, 0.0, 3.0), int project_mode = ORTHO) {
            // set GLOBAL UP to y-axis as default, set look at target as origin(0,0,0)
            this->global_up = Eigen::Vector3d(0.0, 1.0, 0.0);
            this->n = 2.0; this->f = 100.0;
            // FOV angle is initially 60 degrees
            this->theta = (PI/180) * 60;
            // trackball
            this->target = Eigen::Vector3d(0.0, 0.0, 0.0);
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
            double rphi = (PI/180.0)*this->phi, rzeta = (PI/180.0)*this->zeta;
            double y = this->radius * sin(rphi);
            double x = this->radius * sin(rzeta) * cos(rphi);
            double z = this->radius * cos(rzeta) * cos(rphi);

            this->position = Eigen::Vector3d(x, y, z);
        }
        void rotate(int dPhi, int dZeta) {
            this->phi = (this->phi+dPhi+360) % 360;
            this->zeta = (this->zeta+dZeta+360) % 360;
            this->update_camera_pos();
        }
        void zoom(double factor) {
            this->theta *= factor;
        }
        void zoom2D(double factor) {
            this->flatViewMat.col(0)(0) *= factor;
            this->flatViewMat.col(1)(1) *= factor;
            this->flatViewMat.col(3)(0) *= factor;
            this->flatViewMat.col(3)(1) *= factor;
        }
        void pan2D(Eigen::Vector4d delta) {
            this->flatViewMat.col(3) += delta;
        }
        void reset() {
            this->theta = (PI/180) * 60;
            this->phi = 0;
            this->zeta = 0;
            this->update_camera_pos();
        }
        void look_at(GLFWwindow* window, Eigen::Vector3d target = Eigen::Vector3d(0.0, 0.0, 0.0)) {
            this->forward = (this->position - this->target).normalized();
            // special case when forward is parallel to global up
            double dotVal = this->global_up.dot(this->forward);
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
            Eigen::Matrix4d LOOK;
            LOOK <<
            u(0), u(1), u(2), 0.0,
            v(0), v(1), v(2), 0.0,
            w(0), w(1), w(2), 0.0,
            0.0,   0.0, 0.0,  1.0;

            Eigen::Matrix4d AT;
            AT <<
            1.0, 0.0, 0.0, -this->position(0),
            0.0, 1.0, 0.0, -this->position(1),
            0.0, 0.0, 1.0, -this->position(2),
            0.0, 0.0, 0.0, 1.0;

            this->ViewMat = LOOK * AT;

            this->update_project_mat(window);
        }
        Eigen::Vector3d to_world_point(Eigen::Vector3d screen_point) {
            Eigen::Vector3d global_x, global_y, global_z;
            global_x << 1.0, 0.0, 0.0;
            global_y << 0.0, 1.0, 0.0;
            global_z << 0.0, 0.0, 1.0;
            double xworld = screen_point.dot(global_x);
            double yworld = screen_point.dot(global_y);
            double zworld = screen_point.dot(global_z);
            return Eigen::Vector3d(xworld, yworld, zworld);
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
            0.0, 0.0, 0.0, 1.0;

            this->perspect_mat <<
            2*n/(r-l), 0.,      (r+l)/(r-l),    0.,
            0., (2*n)/(t-b),    (t+b)/(t-b),    0.,
            0., 0.,             -(f+n)/(f-n), (-2*f*n)/(f-n),
            0., 0.,             -1.,            0;
        }
        Eigen::Matrix4d get_project_mat() {
            if (this->project_mode == ORTHO) return this->ortho_mat;
            return this->perspect_mat;
        }
};
class Mesh {
    public:
        Eigen::Vector3d color;

        Eigen::Vector4d centroid;
        double r, s, tx, ty;
        Eigen::Vector4d normal;

        std::map<int, Eigen::Vector3d> vid2v;
        std::map<int, Eigen::Vector3d> vid2fv;
        std::vector<int> vids;
        std::vector< std::pair< int, std::pair<int,int> > > nebMeshes;
        int id;

        Eigen::Matrix4d R;
        Eigen::Matrix4d accR;
        Eigen::Matrix4d animeM;
        Mesh* parent;
        std::vector<Mesh*> childs;
        Edge rotEdge;
        Eigen::Vector3d rotAixs;
        Eigen::Vector3d edgeA;
        Eigen::Vector3d edgeB;
        double rotAngle;
        double rotRad;
        double rotSign;
        double rotDot;

        VertexArrayObject VAO;
        VertexBufferObject VBO_P;

        Mesh() {}
        Mesh(int id, Eigen::MatrixXd V, int v1, int v2, int v3, Eigen::Vector3i color=LIGHTGREY) {
            this->id = id;
            this->color = color.cast<double>()/255.;
            this->vids.push_back(v1); this->vids.push_back(v2); this->vids.push_back(v3);
            this->vid2v[v1] = V.col(v1); this->vid2v[v2] = V.col(v2); this->vid2v[v3] = V.col(v3);
            // init flat position
            Eigen::Vector3d zero = Eigen::VectorXd::Zero(3);
            this->vid2fv[v1] = zero; this->vid2fv[v2] = zero; this->vid2fv[v3] = zero;
            this->accR = Eigen::MatrixXd::Identity(4,4);
            this->R = Eigen::MatrixXd::Identity(4,4);
            this->animeM = Eigen::MatrixXd::Identity(4,4);
            this->parent = nullptr;
            this->rotAngle = 0.;
            this->rotRad = 0.;
            this->rotDot = 0.;
            this->rotSign = 1;

            this->VAO.init();
            this->VAO.bind();
            this->VBO_P.init();
            this->updateVBOP();
        }
        void updateVBOP() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3d fV;
            fV << vid2fv[v1], vid2fv[v2], vid2fv[v3];
            this->VBO_P.update(m_to_float(fV));
        }
        Eigen::Matrix3d getFlatV() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3d fV;
            fV << vid2fv[v1], vid2fv[v2], vid2fv[v3];
            return fV;
        }
        Eigen::Matrix3d getV() {
            int v1 = vids[0], v2 = vids[1], v3 = vids[2];
            Eigen::Matrix3d V;
            V << vid2v[v1], vid2v[v2], vid2v[v3];
            return V;
        }
        Mesh(Eigen::MatrixXd V, Eigen::MatrixXd bounding_box, Eigen::Vector3i color=LIGHTGREY) {
            this->r = 0; this->s = 1; this->tx = 0; this->ty = 0;
            this->color = color.cast<double>();
            
            auto a = to_3(V.col(0)), b = to_3(V.col(1)), c = to_3(V.col(2));
            auto tmp = ((b-a).cross(c-a)).normalized();
            this->normal = Eigen::Vector4d(tmp(0), tmp(1), tmp(2), 0.0);
            // Computer the barycenter(centroid) of the Mesh, alphea:beta:gamma = 1:1:1
            Eigen::Vector4d A = V.col(0), B = V.col(1), C = V.col(2);
            this->centroid = (1.0/3)*A + (1.0/3)*B + (1.0/3)*C;
        }
        bool getIntersection(Eigen::Vector4d ray_origin, Eigen::Vector4d ray_direction, Eigen::Vector4d &intersection, Eigen::MatrixXd V) {
            // solve equation:     e + td = a + u(b-a) + v(c-a)
            //                     (a-b)u + (a-c)v + dt = a-e
            Eigen::Vector4d a = V.col(0), b = V.col(1), c = V.col(2);
            Eigen::Vector4d d = ray_direction, e = ray_origin;
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
        Eigen::Vector3d getH(Edge edge) {
            int v1 = edge.first, v2 = edge.second;
            int v3;
            for (int vid: vids) {
                if (vid != v1 && vid != v2) {
                    v3 = vid; break;
                }
            }
            // Eigen::Vector3d aixs = (vid2v[v2]-vid2v[v1]).normalized();
            // Eigen::Vector3d vec = vid2v[v3]-vid2v[v1];
            Eigen::Vector3d aixs = (vid2v[v1]-vid2v[v2]).normalized();
            Eigen::Vector3d vec = vid2v[v3]-vid2v[v2];
            Eigen::Vector3d h = get_vertical_vec(vec, aixs);
            return h;
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
        double sizex, sizey;
        std::map<int, std::map<int, std::vector<int> > > rows;
        Grid(double sizex = 0.03, double sizey = 0.03) {
            this->sizex = sizex; this->sizey = sizey;
        }
        void addItem(Mesh* mesh) {
            Eigen::MatrixXd boundingBox = get_bounding_box_2d(mesh->getFlatV());
            double minx = boundingBox.col(0)(0), maxx = boundingBox.col(1)(0);
            double miny = boundingBox.col(0)(1), maxy = boundingBox.col(1)(1);
            double x = minx;
            while (x < maxx+sizex) {
                double y = miny;
                while (y < maxy+sizey) {
                    int r, c;
                    getCellIdx(x, y, r, c);
                    if (std::find(rows[r][c].begin(), rows[r][c].end(), mesh->id) == rows[r][c].end())
                        rows[r][c].push_back(mesh->id);
                    y += sizey;
                }
                x += sizex;
            }
        }
        void getCellIdx(double x, double y, int &r, int &c) {
            r = int(x/sizex);
            c = int(y/sizey);
        }
        std::set<int> getNearMeshes(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C) {
            // compute bounding box
            Eigen::Matrix3d V;
            V << A, B, C;
            Eigen::MatrixXd boundingBox = get_bounding_box_2d(V);
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
        std::map<int, Mesh*> meshes;
        std::map<std::pair<int, int>, std::vector<int>> edge2meshes;
        std::map<std::pair<int, int>, double> edge2weight;
        Grid* grid;
        std::map<int, int> idx2meshId;

        Eigen::MatrixXd V;
        Eigen::MatrixXd fV;
        Eigen::VectorXi IDX;

        VertexArrayObject VAO;
        VertexBufferObject VBO_P;
        IndexBufferObject IBO_IDX;

        Eigen::MatrixXd ModelMat;
        Eigen::MatrixXd T_to_ori;
        Eigen::Vector4d barycenter;

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
                    meshes[meshId]->nebMeshes.push_back(std::make_pair(nebMeshId, edge));
                }
            }
        }
        bool flattenFirst(int meshId, std::set<int> &flatten) {
            Mesh* mesh = meshes[meshId];
            int v1 = mesh->vids[0], v2 = mesh->vids[1], v3 = mesh->vids[2];
            double v1v2Len = (mesh->vid2v[v1] - mesh->vid2v[v2]).norm();
            // mesh->vid2fv[v1] = Eigen::Vector3d(0., 0., 0.);
            // mesh->vid2fv[v2] = Eigen::Vector3d(0., v1v2Len, 0.);
            mesh->vid2fv[v1] = Eigen::Vector3d(0., 0., -1.);
            mesh->vid2fv[v2] = Eigen::Vector3d(0., v1v2Len, -1.);
            Eigen::Vector3d flatPos;
            if (!flattenVertex(meshId, v3, v1, v2, mesh->vid2fv[v1], mesh->vid2fv[v2], flatPos, flatten)) {
                return false;
            }
            mesh->vid2fv[v3] = flatPos;

            // compute rotate angle
            double rotAngle = 0.;

            Eigen::Vector3d fv1Pos = mesh->vid2fv[v1];
            Eigen::Vector3d fv2Pos = mesh->vid2fv[v2];
            Eigen::Vector3d edgefA = fv1Pos, edgefB = fv2Pos;
            Eigen::Vector3d edgeA = mesh->vid2v[v1], edgeB = mesh->vid2v[v2];
            if (v2 < v1) {
                edgeA = mesh->vid2v[v2]; edgeB = mesh->vid2v[v1];
                edgefA = fv2Pos; edgefB = fv1Pos;
            }
            Eigen::Vector3d fRotAixs = (edgefA-edgefB).normalized();

            mesh->R = get_rotate_mat(rotAngle, edgefA, edgefB);
            mesh->edgeA = edgefA;
            mesh->edgeB = edgefB;
            mesh->rotEdge = std::make_pair(v1, v2);
            mesh->rotAngle = rotAngle;
            mesh->rotAixs = fRotAixs;
            mesh->rotRad = 0.;

            return true;
        }
        FlattenObject(Eigen::MatrixXd V, Eigen::VectorXi IDX, std::vector<bool> &meshFlattened) {
            V.conservativeResize(3, V.cols());
            this->V = V;
            this->fV.resize(4, 0);

            // create V and F matrix
            std::cout << "create meshes" << std::endl;
            // std::vector<Mesh*> meshes;
            for (int i = 0; i < IDX.rows(); i += 3) {
                int meshId = i/3;
                // std::cout << "check id " << i/3 << std::endl;
                if (meshFlattened[i/3]) continue;
                // std::cout << "ok id " << i/3 << std::endl;
                int v1 = IDX(i), v2 = IDX(i+1), v3 = IDX(i+2);
                meshes[meshId] = new Mesh(meshId, V, v1, v2, v3, WHITE);
                // add edge
                addEdge(v1, v2, meshId);
                addEdge(v2, v3, meshId);
                addEdge(v1, v3, meshId);
            }

            // std::cout << "created meshes and edge to meshes" << std::endl;
            // std::cout << "face #: " << meshes.size() << std::endl;
            // std::cout << "edge #: " << edge2meshes.size() << std::endl;

            // add edge field to mesh objects
            for (auto it: meshes) {
                int meshId = it.first;
                Mesh* mesh = it.second;
                int v1 = mesh->vids[0], v2 = mesh->vids[1], v3 = mesh->vids[2];
                addNebMeshes(v1, v2, meshId);
                addNebMeshes(v2, v3, meshId);
                addNebMeshes(v1, v3, meshId);
                // assert(mesh->nebMeshes.size() == 3);
            }
            // std::cout << "created meshes to nebs" << std::endl;

            // new a Regular Grid to boost the overlap checking process.
            grid = new Grid();

            // maximal spaning tree(MST)
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
            // std::cout << "flattened first mesh" << std::endl;
            // std::cout << "mesh flat V" << std::endl;
            // std::cout << meshes[firstMeshId].getFlatV() << std::endl;
            
            // max spanning tree, prime algorithm
            while (!pq.empty()) {
                auto node = pq.top();
                pq.pop();
                Mesh* curMesh = meshes[node.meshId];
                for(auto meshNedge: curMesh->nebMeshes) {
                    int nebMeshId = meshNedge.first;
                    auto edge = meshNedge.second;
                    if (flattened.find(nebMeshId) == flattened.end() && edge2weight[edge] > dist[nebMeshId]) {
                        dist[nebMeshId] = edge2weight[edge];
                        pq.push(Node(edge2weight[edge], edge, curMesh->id, nebMeshId));
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
                    // build MST node connections
                    Mesh* curMesh = meshes[node.meshId];
                    Mesh* preMesh = meshes[node.parentMeshId];
                    curMesh->parent = preMesh;
                    preMesh->childs.push_back(curMesh);
                }
            }

            for (int meshId: flattened) {
                meshFlattened[meshId] = true;
                Eigen::Matrix3d flatV = meshes[meshId]->getFlatV();
                this->fV.conservativeResize(4, fV.cols()+3);
                int last = this->fV.cols();
                this->fV.col(last-3) = to_4_point(flatV.col(0));
                this->fV.col(last-2) = to_4_point(flatV.col(1));
                this->fV.col(last-1) = to_4_point(flatV.col(2));
                this->idx2meshId[last-3] = meshId;
            }

            // update flat position to VBO, compute bounding box
            this->VAO.init();
            this->VAO.bind();
            this->VBO_P.init();
            this->VBO_P.update(m_to_float(this->fV));

            // init model fields
            this->ModelMat = Eigen::MatrixXd::Identity(4,4);
            this->T_to_ori = Eigen::MatrixXd::Identity(4,4);
            this->barycenter = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

            // check tree
            // Mesh* root = meshes.begin()->second;
            // std::queue<Mesh*> q;
            // q.push(root);
            // while (!q.empty()) {
            //     Mesh* cur = q.front();
            //     q.pop();
            //     for (Mesh* child: cur->childs) {
            //         q.push(child);
            //     }
            // }
        }
        
        bool flattenMesh(int preMeshId, int meshId, std::pair<int, int> edge, std::set<int> &flattened) {
            // find the remaining non-flattened vertex
            // std::cout << "enter flattenMesh" << std::endl;
            Mesh* preMesh = meshes[preMeshId];
            Mesh* mesh = meshes[meshId];
            int fv1 = edge.first, fv2 = edge.second;
            int v3;
            for (int vid: mesh->vids) {
                if (vid != fv1 && vid != fv2) {
                    v3 = vid;
                    break;
                }
            }

            // flatten the remaining vertex v3 according to the flat position of v1 and v2
            Eigen::Vector3d fv3Pos;
            if (!flattenVertex(meshId, v3, fv1, fv2, preMesh->vid2fv[fv1], preMesh->vid2fv[fv2], fv3Pos, flattened))
                return false;

            // get flat v1 and flat v2 from pre Mesh
            mesh->vid2fv[fv1] = preMesh->vid2fv[fv1];
            mesh->vid2fv[fv2] = preMesh->vid2fv[fv2];
            mesh->vid2fv[v3] = fv3Pos;

            // compute rotate angle
            Eigen::Vector3d curh = mesh->getH(edge).normalized();
            Eigen::Vector3d preh = preMesh->getH(edge).normalized();
            double rotAngle = 180. - acos(curh.dot(preh)) * 180.0/PI;
            double rotRad = PI-acos(curh.dot(preh));
            double rotDot = -curh.dot(preh);
            double rotSign = 1;
            // std::cout << "curh.dot(preh)" << std::endl;
            // std::cout << curh.dot(preh) << std::endl;
            // std::cout << "rotAngle" << std::endl;
            // std::cout << rotAngle << std::endl;

            Eigen::Vector3d fv1Pos = mesh->vid2fv[fv1];
            Eigen::Vector3d fv2Pos = mesh->vid2fv[fv2];
            Eigen::Vector3d edgefA = fv1Pos, edgefB = fv2Pos;
            Eigen::Vector3d edgeA = mesh->vid2v[fv1], edgeB = mesh->vid2v[fv2];;
            if (fv2 < fv1) {
                edgeA = mesh->vid2v[fv2]; edgeB = mesh->vid2v[fv1];
            }
            Eigen::Vector3d fRotAixs = (edgefA-edgefB).normalized();
            Eigen::Vector3d rotAixs = (edgeA-edgeB).normalized();
            if ((curh.cross(preh)).dot(rotAixs) < 0. ) {
                rotAngle = -rotAngle;
                rotRad = -rotRad;
                rotSign = -1;
            }

            mesh->R = get_rotate_mat(rotAngle, edgefA, edgefB);
            mesh->edgeA = edgefA;
            mesh->edgeB = edgefB;
            mesh->rotEdge = std::make_pair(fv1, fv2);
            mesh->rotAngle = rotAngle;
            mesh->rotRad = rotRad;
            mesh->rotAixs = fRotAixs;
            mesh->rotSign = rotSign;
            mesh->rotDot = rotDot;

            // std::cout << rotAngle << std::endl;

            return true;
        }

        // compute the flat position of v3 according to the flat position of v1 and v2
        // check overlap
        bool flattenVertex(int meshId, int v3, int v1, int v2, Eigen::Vector3d fv1Pos, Eigen::Vector3d fv2Pos, Eigen::Vector3d &fv3Pos, std::set<int> &flattened) {
            Mesh* mesh = meshes[meshId];
            Eigen::Vector3d flat1, flat2;
            
            // use get H to compute fH
            Eigen::Vector3d aixs = (mesh->vid2v[v1]-mesh->vid2v[v2]).normalized();
            Eigen::Vector3d vec = mesh->vid2v[v3]-mesh->vid2v[v2];
            double len = vec.dot(aixs);
            Eigen::Vector3d parallel = len*aixs;
            Eigen::Vector3d hvec = vec-parallel;
            Eigen::Vector3d faixs = (fv1Pos - fv2Pos).normalized();
            Eigen::Vector3d fH = fv2Pos + len * faixs;
            Eigen::Vector3d flatDir = Eigen::Vector3d(-faixs.y(), faixs.x(), 0.).normalized();
            flat1 = fH + hvec.norm() * flatDir;
            flat2 = fH + hvec.norm() * (-flatDir);

            // check overlap
            bool canFlat = false;
            if (!overlap(flat1, fv1Pos, fv2Pos, flattened)) {
                fv3Pos = flat1;
                canFlat = true;
            }
            else {
                if (!overlap(flat2, fv1Pos, fv2Pos, flattened)) {
                    fv3Pos = flat2;
                    canFlat = true;
                }
            }

            return canFlat;
        }

        bool overlap(Eigen::Vector3d flatPos, Eigen::Vector3d fv1Pos, Eigen::Vector3d fv2Pos, std::set<int> &flattened) {
            // check if any vertices of a flat Triangle inside the other flat Triangle
            // get all near meshes and combine them to one vector
            std::set<int> nearMeshes = grid->getNearMeshes(flatPos, fv1Pos, fv2Pos);
            for (int meshId: nearMeshes) {
                Eigen::Matrix3d meshfV = meshes[meshId]->getFlatV();
                if (isInside(flatPos, meshfV)) return true;
                Eigen::Vector3d center = (flatPos+fv1Pos+fv2Pos)/3.;
                if (isInside(center, meshfV)) return true;

                Eigen::Matrix3d curMeshfV;
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
        bool lineCross(Eigen::Vector3d a, Eigen::Vector3d b, int meshId) {
            Mesh* mesh = meshes[meshId];
            int ov1, ov2, ov3;
            ov1 = mesh->vids[0]; ov2 = mesh->vids[1]; ov3 = mesh->vids[2];
            if (lineCross(a, b, mesh->vid2fv[ov1], mesh->vid2fv[ov2])) return true;
            if (lineCross(a, b, mesh->vid2fv[ov2], mesh->vid2fv[ov3])) return true;
            if (lineCross(a, b, mesh->vid2fv[ov1], mesh->vid2fv[ov3])) return true;
            return false;
        }
        bool lineCross(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d) {
            // overlap?
            if (((a-c).norm() < ESP || (a-d).norm() < ESP) && ((b-c).norm() < ESP || (b-d).norm() < ESP)) {
                return false;
            }
            // parallel?
            Eigen::Vector3d AB = b-a;
            Eigen::Vector3d CD = d-c;
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
        bool isInside(Eigen::Vector3d flatPos, Eigen::Matrix3d meshfV) {
            Eigen::Vector3d a, b, c;
            a = meshfV.col(0); b = meshfV.col(1); c = meshfV.col(2);

            Eigen::Matrix3d M;
            Eigen::Vector3d R;
            M << a(0),b(0),c(0),  a(1),b(1),c(1), 1,1,1;
            R << flatPos(0), flatPos(1), 1;
            Eigen::Vector3d x = M.colPivHouseholderQr().solve(R);

            return x(0)-0. > ESP && x(1)-0. > ESP && x(2)-0. > ESP;
        }
        void translate(Eigen::Vector4d delta) {
            Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
            T.col(3)(0) = delta(0); T.col(3)(1) = delta(1); T.col(3)(2) = delta(2);
            this->update_Model_Mat(T, true);
        }
        void scale(double factor) {
            // double factor = 1+delta;
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(4, 4);
            S.col(0)(0) = factor; S.col(1)(1) = factor; S.col(2)(2) = 1.;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);
            S = (2*I-this->T_to_ori)*S*(this->T_to_ori);
            this->update_Model_Mat(S, false);
        }
        void rotate(double degree, Eigen::Matrix4d rotateMat) {
            double r = degree*PI/180.0;
            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
            R.col(0)(0) = std::cos(r); R.col(0)(1) = std::sin(r);
            R.col(1)(0) = -std::sin(r); R.col(1)(1) = std::cos(r);
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);
            R = (2*I-this->T_to_ori)*rotateMat*(this->T_to_ori);
            this->update_Model_Mat(R, false);
        }
        void update_Model_Mat(Eigen::MatrixXd M, bool left) {
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
                Eigen::MatrixXd mesh_fV(4, 3);
                mesh_fV.col(0) = fV.col(i);
                mesh_fV.col(1) = fV.col(i+1);
                mesh_fV.col(2) = fV.col(i+2);
                
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
        void adjustSize(Eigen::MatrixXd boundingBox) {
            double maxx = boundingBox.col(1)(0), maxy = boundingBox.col(1)(1);
            double minx = boundingBox.col(0)(0), miny = boundingBox.col(0)(1);
            Eigen::MatrixXd curBox = get_bounding_box_2d(this->fV);
            this->barycenter = (curBox.col(0) + curBox.col(1))/2.0;
            this->barycenter(2) = 0; this->barycenter(3) = 1;

            // Computer the translate Matrix from barycenter to the origin
            Eigen::Vector4d delta = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0) - this->barycenter;
            T_to_ori.col(3)(0) = delta(0); T_to_ori.col(3)(1) = delta(1); T_to_ori.col(3)(2) = delta(2);

            // adjust inital position according to bounding box
            double scale_factor = fmin(1.0/(maxx-minx), 1.0/(maxy-miny));
            this->translate(delta);
            this->scale(scale_factor);
        }
};
class _3dObject {
    public:
        Eigen::MatrixXd box;
        Eigen::MatrixXd ModelMat;
        Eigen::MatrixXd ModelMat_T;
        Eigen::MatrixXd Adjust_Mat;
        Eigen::MatrixXd T_to_ori;
        Eigen::Vector4d barycenter;
        Eigen::VectorXi IDX;
        Eigen::MatrixXd V;
        Eigen::MatrixXd C;
        Eigen::MatrixXd Normals;

        int render_mode;
        double r, s, tx, ty;
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
            Eigen::MatrixXd V, C;
            Eigen::VectorXi IDX;
            loadMeshfromOFF(off_path, V, C, IDX);
            C = Eigen::MatrixXd(3, V.cols());
            // int color_idx = rand() % colors.size();
            Eigen::Vector3i color = colors[color_idx];
            for (int i = 0; i < V.cols(); i++) {
                C.col(i) = color.cast<double>();
            }
            //compute the bouncing box
            box = get_bounding_box(V);
            //create class Mesh for each mech
            this->initial(V, C, IDX, box);
        }
        void initial(Eigen::MatrixXd V, Eigen::MatrixXd C, Eigen::VectorXi IDX, Eigen::MatrixXd bounding_box) {
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
            this->VBO_P.update(m_to_float(V));
            this->VBO_C.init();
            this->VBO_C.update(m_to_float(C/255.0));
            this->VBO_N.init();
            this->IBO_IDX.init();
            this->IBO_IDX.update(IDX);

            this->ModelMat = Eigen::MatrixXd::Identity(4,4);
            this->ModelMat_T = Eigen::MatrixXd::Identity(4,4);
            this->T_to_ori = Eigen::MatrixXd::Identity(4,4);
            this->r = 0; this->s = 1; this->tx = 0; this->ty = 0;
            this->render_mode = 0;
            this->barycenter = (bounding_box.col(0) + bounding_box.col(1))/2.0;

            // Computer the translate Matrix from barycenter to the origin
            Eigen::Vector4d delta = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0) - this->barycenter;
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
                Eigen::MatrixXd Vblock(4, 3);
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
                Eigen::Vector4d normal(0., 0., 0., 0.);
                for (Mesh* mesh: this->edges[i]) {
                    normal += mesh->normal;
                }
                this->Normals.col(i) = normal.normalized();
            }
            this->VBO_N.update(m_to_float(this->Normals));

            std::cout << "meshes # = " << this->meshes.size() << std::endl;
            std::cout << "finish" << std::endl;

            // create flatten object
            // this->flattenObj = nullptr;
            // this->flattenObjs.resize(10);
            this->flatten();
        }
        void initial_adjust(Eigen::MatrixXd bounding_box) {
            double maxx = bounding_box.col(1)(0), maxy = bounding_box.col(1)(1), maxz = bounding_box.col(1)(2);
            double minx = bounding_box.col(0)(0), miny = bounding_box.col(0)(1), minz = bounding_box.col(0)(2);
            double scale_factor = fmin(1.0/(maxx-minx), fmin(1.0/(maxy-miny), 1.0/(maxz-minz)));
            this->scale(scale_factor);
        }
        bool hit(Eigen::Vector4d ray_origin, Eigen::Vector4d ray_direction, double &dist) {

            bool intersected = false;
            int cnt = 0;
            int selectedMeshId;
            for (auto mesh : this->meshes) {
                Eigen::Vector4d intersection;
                Eigen::MatrixXd mesh_V(4,3);
                int i = this->IDX(cnt*3), j = this->IDX(cnt*3+1), k = this->IDX(cnt*3+2);
                mesh_V << this->V.col(i), this->V.col(j), this->V.col(k);
                mesh_V = this->ModelMat*mesh_V;

                // the ray is parallel to the Mesh, no solution
                Eigen::Vector4d world_normal = ((this->ModelMat.inverse()).transpose()*mesh->normal).normalized();
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
        void translate(Eigen::Vector4d delta) {
            // delta = this->Adjust_Mat.inverse()*delta;
            this->tx += delta(0); this->ty += delta(1);
            Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
            T.col(3)(0) = delta(0); T.col(3)(1) = delta(1); T.col(3)(2) = delta(2);
            Eigen::MatrixXd T_T = Eigen::MatrixXd::Identity(4, 4);
            T_T.col(3)(0) = -delta(0); T_T.col(3)(1) = -delta(1); T_T.col(3)(2) = -delta(2);
            this->update_Model_Mat(T, T_T, true);
        }
        void scale(double factor) {
            // double factor = 1+delta;
            this->s *= factor;
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(4, 4);
            S.col(0)(0) = factor; S.col(1)(1) = factor; S.col(2)(2) = factor;
            Eigen::MatrixXd S_T = Eigen::MatrixXd::Identity(4, 4);
            S_T.col(0)(0) = 1.0/factor; S_T.col(1)(1) = 1.0/factor; S_T.col(2)(2) = 1.0/factor;

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);
            S = (2*I-this->T_to_ori)*S*(this->T_to_ori);
            S_T = (2*I-this->T_to_ori)*S_T*(this->T_to_ori);

            this->update_Model_Mat(S, S_T, false);
        }
        void rotate(double degree, Eigen::Matrix4d rotateMat) {
            double r = degree*PI/180.0;
            this->r += r;
            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
            R.col(0)(0) = std::cos(r); R.col(0)(1) = std::sin(r);
            R.col(1)(0) = -std::sin(r); R.col(1)(1) = std::cos(r);
            Eigen::MatrixXd R_T = Eigen::MatrixXd::Identity(4, 4);
            R_T.col(0)(0) = std::cos(r); R_T.col(0)(1) = -std::sin(r);
            R_T.col(1)(0) = std::sin(r); R_T.col(1)(1) = std::cos(r);

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);
            R = (2*I-this->T_to_ori)*rotateMat*(this->T_to_ori);
            R_T = (2*I-this->T_to_ori)*rotateMat.inverse()*(this->T_to_ori);

            this->update_Model_Mat(R, R_T, false);
        }
        void update_Model_Mat(Eigen::MatrixXd M, Eigen::MatrixXd M_T, bool left) {
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
                // check tree
                std::cout << "check tree in 3d object flatten" << std::endl;
                for (FlattenObject& flatObj: this->flattenObjs) {
                    Mesh* root = flatObj.meshes.begin()->second;
                    std::queue<Mesh*> q;
                    q.push(root);
                    std::cout << "tree" << std::endl;
                    while (!q.empty()) {
                        Mesh* cur = q.front();
                        q.pop();
                        std::cout << cur->id << std::endl;
                        for (Mesh* child: cur->childs) {
                            q.push(child);
                        }
                    }
                }

                flattedCnt = 0;
                for (int i = 0; i < meshFlattened.size(); i++) {
                    flattedCnt += meshFlattened[i];
                }
            }

            // scale all islands with a same ratio to fit the window
            std::vector<Eigen::Matrix2d> islandsBoxs;
            Eigen::MatrixXd boundingBox(2, 2);
            double deltaY = 0.;
            for (FlattenObject &flatObj: this->flattenObjs) {
                Eigen::MatrixXd box = get_bounding_box_2d(flatObj.fV);
                islandsBoxs.push_back(box);
                if (box.col(1)(1)-box.col(0)(1) > deltaY) {
                    deltaY = box.col(1)(1)-box.col(0)(1);
                    boundingBox = box;
                }
            }
            std::cout << "island # = " << this->flattenObjs.size() << std::endl;
            for (FlattenObject &flatObj: this->flattenObjs) {
                std::cout << "mesh # = " << flatObj.fV.cols()/3 << std::endl;
            }

            // arrange the layout of islands on paper
            double maxW = 0.;
            for (auto box: islandsBoxs) {
                maxW = fmax(maxW, box.col(1).x()-box.col(0).x());
            }
            double paperL = 0., paperT = 0., paperR = maxW, paperB = 0.;
            double curX = paperL, curY = paperT;
            double margin = 0.1;
            for (int i = 0; i < this->flattenObjs.size(); i++) {
                FlattenObject &flatObj = this->flattenObjs[i];
                Eigen::Matrix2d box = islandsBoxs[i];
                double w = box.col(1).x()-box.col(0).x(), h = box.col(1).y()-box.col(0).y();
                if (curX+w+margin > paperR) {
                    curX = paperL;
                    curY = paperB;
                }
                islandMoveTo(curX, curY, box, flatObj);
                curX += w+margin;
                paperB = fmin(paperB, curY-(h+margin));
            }

            // scale the whole paper to fit the window
            double scaleFactor = fmin(1.0/(paperT-paperB), 1.0/(paperR-paperL));
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(4, 4);
            S.col(0)(0) = scaleFactor; S.col(1)(1) = scaleFactor; S.col(2)(2) = scaleFactor;
            for (FlattenObject &flatObj: this->flattenObjs) {
                flatObj.ModelMat = S*flatObj.ModelMat;
            }

            // move paper center to the center of the screen
            Eigen::Vector4d paperCenter(scaleFactor*(paperL+paperR)/2.0, scaleFactor*(paperT+paperB)/2.0, 0., 1.);
            Eigen::Vector4d delta = Eigen::Vector4d(0., 0., 0., 1.)-paperCenter;
            for (FlattenObject &flatObj: this->flattenObjs) {
                flatObj.translate(delta);
            }
        }
        void islandMoveTo(double l, double t, Eigen::Matrix2d boundBox, FlattenObject &flatObj) {
            Eigen::Vector2d leftTop = Eigen::Vector2d(l, t);
            double bminx = boundBox.col(0).x(), bmaxy = boundBox.col(1).y();
            Eigen::Vector4d bleftTop = Eigen::Vector4d(bminx, bmaxy, 0., 1.);
            bleftTop = flatObj.ModelMat*bleftTop;
            Eigen::Vector2d delta = leftTop - Eigen::Vector2d(bleftTop.x(), bleftTop.y());
            flatObj.translate(Eigen::Vector4d(delta(0), delta(1), 0., 0.));
        }
};
class _3dObjectBuffer {
    public:
        std::vector<_3dObject*> _3d_objs;
        _3dObject* selected_obj;
        FlattenObject* selected_flat_obj;
        int color_idx;

        _3dObjectBuffer() {
            selected_obj = nullptr;
            selected_flat_obj = nullptr;
            color_idx = 0;
        }
        void add_object(std::string off_path) {
            this->color_idx = (this->color_idx)%colors.size();
            this->color_idx++;
            _3d_objs.push_back(new _3dObject(off_path, this->color_idx));
        }
        bool hit(int subWindow, Eigen::Vector4d ray_origin, Eigen::Vector4d ray_direction, double &ret_dist, int mode=CILCK_ACTION) {
            double min_dist = DIST_MAX;
            _3dObject* selected = nullptr;
            FlattenObject* selected_flat = nullptr;
            if (subWindow == LEFTSUBWINDOW) {
                // find the hitted 3D object if any
                for (auto obj: _3d_objs) {
                    double dist = DIST_MAX;
                    if (obj->hit(ray_origin, ray_direction, dist)) {
                        if (selected == nullptr || min_dist > dist) {
                            min_dist = dist;
                            selected = obj;
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
        bool translate(Eigen::Vector4d delta) {
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
        bool rotate(double degree, Eigen::Vector3d rotateAixs) {
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
        Eigen::Matrix4d get_rotate_mat(double angle, Eigen::Vector3d rotateAixs) {
            double r = (PI/180) * (angle/2);
            double x = rotateAixs.x() * sin(r);
            double y = rotateAixs.y() * sin(r);
            double z = rotateAixs.z() * sin(r);
            double w = cos(r);
            Eigen::Matrix4d rotate_mat;
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
class Player {
    public:
        bool playing;
        double frames;
        int frame;
        std::queue<Mesh*> waitlist;

        void init(_3dObject* obj3d) {
            // FlattenObject& flatObj = obj3d->flattenObjs[0];
            for (FlattenObject &flatObj: obj3d->flattenObjs) {
                waitlist.push(flatObj.meshes.begin()->second);
            }
            frames = 10.;
            frame = 0;
            playing = true;
        }
        void nextFrame() {
            updateAnimateMats();
        }
        void updateAnimateMats() {
            // std::cout << "update animate matrix" << std::endl;
            frame += 1;
            const double dt = (double)frame/frames;
            if (dt > 1.) {
                frame = 0;
                Mesh* root = waitlist.front();
                waitlist.pop();
                for (Mesh* child: root->childs) {
                    child->accR = root->animeM;
                    waitlist.push(child);
                }
            }
            // BFS update
            if (!waitlist.empty()) {
                Mesh* root = waitlist.front();
                std::queue<Mesh*> q;
                q.push(root);
                Eigen::Matrix4d R = get_rotate_mat(dt*root->rotRad, root->edgeA, root->edgeB);
                root->animeM = root->accR * R;
                while (!q.empty()) {
                    Mesh* cur = q.front();
                    q.pop();
                    for (Mesh* child: cur->childs) {
                        child->animeM = root->animeM;
                        q.push(child);
                    }
                }
            }
            else {
                playing = false;
                std::cout << "end keyframe playing" << std::endl;
            }
        }
};
class CameraBuffer {
    public:
        std::vector<Camera*> cameras;
        int focusWindow;
        CameraBuffer(GLFWwindow* window, int size) {
            for (int i = 0; i < size; i++)
                cameras.push_back(new Camera(window));
            focusWindow = 0;
        }
        Camera* getCamera() {
            return cameras[focusWindow];
        }
        void updateWindowScale(GLFWwindow* window) {
            for (auto camera: cameras) {
                camera->update_project_mat(window);
                camera->look_at(window);
            }
        }
};

_3dObjectBuffer* _3d_objs_buffer;
std::vector<_3dObject*>* _3d_objs;
// Camera *camera;
CameraBuffer* camera_buf;
Player player = Player();

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
    std::string flat_svg_str = _3d_objs_buffer->export_flat_svg(camera_buf->getCamera());
    svg_str = replace_all(svg_str, "$TG", flat_svg_str);
    // Save svg to file
    std::ofstream svg_file;
    svg_file.open (EXPORT_PATH);
    svg_file << svg_str;
    svg_file.close();
}

Eigen::Vector4d get_click_position(GLFWwindow* window, int &subWindow) {
    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    Eigen::Vector4d p_screen;
    // click on right screen
    if (xpos > width/2) {
        subWindow = RIGHTSUBWINDOW;
        p_screen = Eigen::Vector4d((xpos-width/2)*2,height-1-ypos,0.0,1.0);
    }
    // click on left screen
    else {
        subWindow = LEFTSUBWINDOW;
        p_screen = Eigen::Vector4d(xpos*2,height-1-ypos,0.0,1.0);
    }
    Camera* camera = camera_buf->getCamera();
    Eigen::Vector4d p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,-camera->n,1.0);
    Eigen::Vector4d p_camera = camera->get_project_mat().inverse() * p_canonical;
    if (fabs(p_camera(3)-1.0) > 0.001) {
        p_camera = p_camera/p_camera(3);
    }

    Eigen::Vector4d p_world;
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
    Eigen::Vector4d click_point = get_click_position(window, subWindow);
    camera_buf->focusWindow = subWindow;
    Camera* camera = camera_buf->getCamera();

    // Update the position of the first vertex if the left button is pressed
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        Eigen::Vector4d ray_origin = click_point;
        // orth projection
        Eigen::Vector4d ray_direction;
        if (subWindow == RIGHTSUBWINDOW) {
            ray_direction = Eigen::Vector4d(0., 0., -1., 0.);
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
        double dist = 0;
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
    Eigen::Vector4d click_point = get_click_position(window, subWindow);

    if (drag) {
        Eigen::Vector4d delta = click_point-pre_cursor_point;
        _3d_objs_buffer->translate(delta);
        pre_cursor_point = click_point;
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Camera* camera = camera_buf->getCamera();
    switch (key)
    {
        // export svg
        case  GLFW_KEY_0:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "export SVG");
                export_svg(window);
            }
            break;
        // import an object
        case  GLFW_KEY_1:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "import an object");
                _3d_objs_buffer->add_object(CUSTOM_OFF_PATH);
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
                // if (_3d_objs_buffer->translate(Eigen::Vector4d(0., 0.2, 0., 0.))) {
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
                glfwSetWindowTitle (window, "right-subwindow zoom in 10%");
                camera->zoom(0.9);
                camera->look_at(window);
            }
            break;
        case GLFW_KEY_MINUS:
            if (action == GLFW_PRESS) {
                glfwSetWindowTitle (window, "left-subwindow zoom out 10%");
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
                if (_3d_objs_buffer->selected_obj != nullptr && !player.playing) {
                    glfwSetWindowTitle (window, "flatten the selected 3d object");
                    _3d_objs_buffer->selected_obj->flatten();
                    _3d_objs_buffer->selected_flat_obj = nullptr;
                }
            }
            break;
        // play animation
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS) {
                if (_3d_objs_buffer->selected_obj != nullptr) {
                    glfwSetWindowTitle (window, "play animation");
                    player.init(_3d_objs_buffer->selected_obj);
                }
            }
            break;
        default:
            break;
    }
}

int main(void)
{
    std::cout << std::setprecision(10);

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
            "#version 410 core\n"
                    "in vec4 position;"
                    "uniform vec3 color;"
                    "uniform mat4 ViewMat;"
                    "uniform mat4 ModelMat;"
                    "uniform mat4 ProjectMat;"
                    "uniform mat4 AnimateT;"
                    "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    gl_Position = ProjectMat*ViewMat*ModelMat*AnimateT*position;"
                    "    f_color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 410 core\n"
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
    // camera = new Camera(window);
    camera_buf = new CameraBuffer(window, 2);

    // bind light
    // auto light = _3d_objs_buffer->ray_tracer->lights[0];
    // glUniform4fv(program.uniform("light.position"), 1, light->position.data());
    // glUniform3fv(program.uniform("light.intensities"), 1, light->intensity.data());
    // bind special colors
    Eigen::Vector3d special_color = (SELCET_COLOR.cast<double>())/255.0;
    // glUniform3fv(program.uniform("selectedColor"), 1, special_color.data());
    special_color = (BLACK.cast<double>())/255.0;
    // special_color = (WHITE.cast<double>())/255.0;
    glUniform3fv(program.uniform("lineColor"), 1, v_to_float(special_color).data());
    Eigen::MatrixXd I44 = Eigen::MatrixXd::Identity(4,4);

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
        camera_buf->updateWindowScale(window);
        
        // define render color
        Eigen::Vector3d white = (WHITE.cast<double>())/255.0;
        Eigen::Vector3d red = (RED.cast<double>())/255.0;

        int WindowWidth, WindowHeight;
        glfwGetWindowSize(window, &WindowWidth, &WindowHeight);

        // right screen
        glViewport(WindowWidth, 0, WindowWidth, WindowHeight*2);
        // Send the View Mat to Vertex Shader
        Camera* rightCam = camera_buf->cameras[1];
        Eigen::Vector4d rightCamPos = to_4_point(rightCam->position);
        glUniform4fv(program.uniform("viewPosition"), 1, v_to_float(rightCamPos).data());
        glUniformMatrix4fv(program.uniform("ViewMat"), 1, GL_FALSE, m_to_float(rightCam->ViewMat).data());
        glUniformMatrix4fv(program.uniform("ProjectMat"), 1, GL_FALSE, m_to_float(rightCam->get_project_mat()).data());

        if (player.playing) {
            player.nextFrame();
        }
        for (auto obj: _3d_objs_buffer->_3d_objs) {
            // prepare
            for (FlattenObject &flatObj: obj->flattenObjs) {
                glUniformMatrix4fv(program.uniform("ModelMat"), 1, GL_FALSE, m_to_float(flatObj.ModelMat).data());

                flatObj.VAO.bind();
                program.bindVertexAttribArray("position", flatObj.VBO_P);
                glUniform3fv(program.uniform("color"), 1, v_to_float(white).data());
                for (int i = 0; i < flatObj.fV.cols(); i += 3) {
                    // get animation model matrix
                    int meshId = flatObj.idx2meshId[i];
                    Mesh* mesh = flatObj.meshes[meshId];
                    Eigen::Matrix4d AnimateT = mesh->animeM;
                    // std::cout << "AnimateT" << std::endl;
                    // std::cout << AnimateT << std::endl;
                    glUniformMatrix4fv(program.uniform("AnimateT"), 1, GL_FALSE, m_to_float(AnimateT).data());

                    glUniform1i(program.uniform("isLine"), 1);
                    glDrawArrays(GL_LINE_LOOP, i, 3);
                    glUniform1i(program.uniform("isLine"), 0);
                    glDrawArrays(GL_TRIANGLES, i, 3);
                }
            }
        }

        // left screen
        glViewport(0, 0, WindowWidth, WindowHeight*2);
        // Send the View Mat to Vertex Shader
        Camera* leftCam = camera_buf->cameras[0];
        Eigen::Vector4d leftCamPos = to_4_point(leftCam->position);
        glUniform4fv(program.uniform("viewPosition"), 1, v_to_float(leftCamPos).data());
        glUniformMatrix4fv(program.uniform("ViewMat"), 1, GL_FALSE, m_to_float(leftCam->ViewMat).data());
        glUniformMatrix4fv(program.uniform("ProjectMat"), 1, GL_FALSE, m_to_float(leftCam->get_project_mat()).data());
        glUniformMatrix4fv(program.uniform("ViewMat"), 1, GL_FALSE, m_to_float(leftCam->ViewMat).data());
        glUniformMatrix4fv(program.uniform("AnimateT"), 1, GL_FALSE, m_to_float(I44).data());

        for (auto obj: _3d_objs_buffer->_3d_objs) {
            obj->VAO.bind();
            program.bindVertexAttribArray("position",obj->VBO_P);
            glUniformMatrix4fv(program.uniform("ModelMat"), 1, GL_FALSE, m_to_float(obj->ModelMat).data());
            for (int i = 0; i < obj->meshes.size(); i++) {
                if (obj->selectedMeshes.find(i) != obj->selectedMeshes.end()) {
                    glUniform3fv(program.uniform("color"), 1, v_to_float(red).data());
                }
                else {
                    glUniform3fv(program.uniform("color"), 1, v_to_float(white).data());
                }
                glUniform1i(program.uniform("isLine"), 1);
                glDrawElements(GL_LINE_LOOP, 3, GL_UNSIGNED_INT, (void*)(sizeof(int)* (i*3)));
                glUniform1i(program.uniform("isLine"), 0);
                glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, (void*)(sizeof(int)* (i*3)));
            }
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);
        
        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}