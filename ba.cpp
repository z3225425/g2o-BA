#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>
#include<eigen3/Eigen/Core>



#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



using namespace std;
using namespace g2o;




int main()
{
    fstream fin1("../p2d.txt");
    fstream fin2("../p3d.txt");
    vector<Eigen::Vector2d>point2;
    vector<Eigen::Vector3d>point3;
    while(!fin1.eof()){
        Eigen::Vector2d p2;
         for(int i=0;i<2;i++)
         {

             fin1>>p2[i];
         }
         point2.push_back(p2);
    }
    while(!fin2.eof()){
        Eigen::Vector3d p3;
        for(int i=0;i<3;i++)
        {

            fin2>>p3[i];
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
    
    g2o::SparseOptimizer    optimizer;
         
         g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
        
         g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
       
         g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );

         optimizer.setAlgorithm( algorithm );
        optimizer.setVerbose( false );



    g2o::VertexSE3Expmap *vertex2=new g2o::VertexSE3Expmap();
    vertex2->setId(0);
    vertex2->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(vertex2);

    for(int i=0;i<point2.size();i++)
  {
        g2o::VertexSBAPointXYZ *pointXYZ=new g2o::VertexSBAPointXYZ();
        pointXYZ->setId(i+1);
        pointXYZ->setFixed(true);
      pointXYZ->setMarginalized(true);
        pointXYZ->setEstimate(point3[i]);

        optimizer.addVertex(pointXYZ);
    }

    g2o::CameraParameters* cam=new g2o::CameraParameters(520.9,Eigen::Vector2d(325.1,249.7),0);
    cam->setId(0);
    optimizer.addParameter(cam);






   for(int i=0;i<point2.size();i++)
    {
        g2o::EdgeProjectXYZ2UV *edge=new g2o::EdgeProjectXYZ2UV();

        edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+1)));
        edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
        edge->setMeasurement(point2[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0,0);
        optimizer.addEdge(edge);

    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);
   g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(0) );
   Eigen::Isometry3d pose = v->estimate();
   cout<<pose.matrix();
 
}
