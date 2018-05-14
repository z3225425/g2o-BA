#include <iostream>
#include<fstream>
#include<g2o/core/block_solver.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_binary_edge.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>

#include"sophus/se3.h"
#include"sophus/so3.h"
using Sophus::SE3;
using Sophus::SO3;
using namespace std;
using namespace Eigen;

double fx=520.9;
double fy=521.0;
double cx=325.1;
double cy=249.7;


class  VertexSE :public g2o::BaseVertex<6,SE3>
        {
public:
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            bool read(istream & is){};
           bool write(ostream & out)const {};
            virtual  void setToOriginImpl() override
                {
                    _estimate=Sophus::SE3();
                }
             virtual void oplusImpl(const double *update) override
             {
                 SE3 up(SO3(update[3],update[4],update[5]),Eigen::Vector3d(update[0],update[1],update[2]));
                 _estimate=up*_estimate;

             }
        };

class points3d :public g2o::BaseVertex<3,Eigen::Vector3d>
        {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
     bool read(istream & is){};
      bool write(ostream &out) const {};
    virtual  void setToOriginImpl() override
    {
        _estimate=Eigen::Vector3d::Identity();
    }
    virtual void oplusImpl(const double *update) override
    {
          Eigen::Vector3d up=Eigen::Vector3d(update);
        _estimate+=up;

    }

        };

class Edgeba: public g2o::BaseBinaryEdge<2,Eigen::Vector2d,VertexSE,points3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    bool read(istream & is){};
    bool write(ostream &out) const {};
    virtual void computeError() override
    {
        Sophus::SE3 v1=(static_cast<VertexSE*>(_vertices[0])->estimate());
        Eigen::Vector3d v2=(static_cast<points3d*>(_vertices[1])->estimate());
        Eigen::Vector3d point=v1*v2;
         _error(0,0)=_measurement(0)-(fx*point(0,0)/point(2,0)+cx);
         _error(1,0)=_measurement(1)-(fy*point(1,0)/point(2,0)+cy);
    }


};

int main() {
    fstream fin1("../p2d.txt");
    fstream fin2("../p3d.txt");
    vector<Eigen::Vector2d> point2;
    vector<Eigen::Vector3d> point3;
    while (!fin1.eof()) {
        Eigen::Vector2d p2;
        for (int i = 0; i < 2; i++) {

            fin1 >> p2[i];
        }
        point2.push_back(p2);
    }
    while (!fin2.eof()) {
        Eigen::Vector3d p3;
        for (int i = 0; i < 3; i++) {

            fin2 >> p3[i];
        }
        point3.push_back(p3);
    }

    /*
     typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
      Block::LinearSolverType *linearsolve=new g2o::LinearSolverDense<Block::PoseMatrixType>();
      Block *block_solver=new Block(linearsolve);
      g2o::OptimizationAlgorithmLevenberg *solver=new g2o::OptimizationAlgorithmLevenberg(block_solver);

      g2o::SparseOptimizer optimizer;
      optimizer.setAlgorithm(solver);
     */
    g2o::SparseOptimizer optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    // 6*3 的参数
    g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linearSolver);
    // L-M 下降
    g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(true);


    VertexSE *vertex2 = new VertexSE();
    vertex2->setId(0);
    vertex2->setEstimate(SE3());
    optimizer.addVertex(vertex2);

    for (int i = 0; i < point2.size(); i++) {
        points3d *pointXYZ = new points3d();
        pointXYZ->setId(i + 1);
        pointXYZ->setFixed(true);
        pointXYZ->setMarginalized(true);
        pointXYZ->setEstimate(point3[i]);

        optimizer.addVertex(pointXYZ);
    }


    for (int i = 0; i < point2.size(); i++) {
        Edgeba *edge = new Edgeba();

        edge->setVertex(0, dynamic_cast<VertexSE *> (optimizer.vertex(0)));
        edge->setVertex(1, dynamic_cast<points3d *> (optimizer.vertex(i+1)));
        edge->setMeasurement(point2[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        optimizer.addEdge(edge);

    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    VertexSE *v = dynamic_cast<VertexSE *>( optimizer.vertex(0));
    Sophus::SE3 pose = v->estimate();
    cout << pose.matrix();
}
