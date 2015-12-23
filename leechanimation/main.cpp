//
//  main.cpp
//  leechanimation
//
//  Created by Michael Klein on 12/20/15.
//  Copyright © 2015 Michael Klein. All rights reserved.
//


// TODO: modify mesh parameters to better fit this case


// Utilizes Eigen 3 and PCL 1.8

//#include <iostream>
//
//int main(int argc, const char * argv[]) {
//    // insert code here...
//    std::cout << "Hello, World!\n";
//    return 0;
//}

#include <iostream>
#include <fstream>
#include <iomanip>
#include </usr/local/include/eigen3/Eigen/Dense>
#include </usr/local/include/eigen3/unsupported/Eigen/MatrixFunctions>
#include </usr/local/include/pcl-1.8/pcl/point_cloud.h>
#include </usr/local/include/pcl-1.8/pcl/point_types.h>
#include </usr/local/include/pcl-1.8/pcl/io/pcd_io.h>
#include </usr/local/include/pcl-1.8/pcl/ModelCoefficients.h>
#include </usr/local/include/pcl-1.8/pcl/filters/project_inliers.h>
#include </usr/local/include/pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include </usr/local/include/pcl-1.8/pcl/features/normal_3d.h>
#include </usr/local/include/pcl-1.8/pcl/surface/gp3.h>
#include </usr/local/include/pcl-1.8/pcl/io/vtk_io.h>

using namespace Eigen;
using namespace std;
//using namespace pcl;

typedef Matrix<double, 24, 24> Matrix24f;
typedef Matrix<double, 24, 1> Vector24f;

Vector24f binvec(int n0){
    Vector24f out;
    int n = n0;
    for (int i = 0; i != 24; i++) {
        out(i) = (double) (n & 1);
        n >>= 1;
    }
    return out;
}

Vector24f binvec2(size_t n0){
    Vector24f out;
    size_t n = n0;
    for (int i = 0; i != 24; i++) {
        out(i) = (double) (n & 1);
        n >>= 1;
    }
    return out;
}

Vector3d extract3d(Vector24f v24){
    Vector3d v3;
    v3(0) = v24(0);
    v3(1) = v24(1);
    v3(2) = v24(2);
    return v3;
}

Vector3d transd3d(int n, Matrix24f lmat, Matrix24f tmat){
    Vector24f v = binvec(n);
    v = lmat*v;
    v = tmat*v;
    return extract3d(v);
}

#define DIMOFFSET 100

bool checkv(Vector3d v){
    return (v(0) >= DIMOFFSET) && (v(1) >= DIMOFFSET) && (v(2) >= DIMOFFSET);
}

//void printable(ofstream& vectorz, int n, Matrix24f lmat, Matrix24f tmat){
//    Vector24f v0 = binvec(n);
//    v0 = lmat*v0;
//    v0 = tmat*v0;
//    Vector3d v1 = extract3d(v0);
//    if (checkv(v1)) {
//        vectorz << std::fixed << std::setprecision(10) << (v1(0) + DIMOFFSET);
//        vectorz << " ";
//        vectorz << std::fixed << std::setprecision(10) << (v1(1) + DIMOFFSET);
//        vectorz << " ";
//        vectorz << std::fixed << std::setprecision(10) << (v1(2) + DIMOFFSET);
//        vectorz << "\n";
//    }
//}

//void writevs(string filename, int howmany, Matrix24f lmat, Matrix24f tmat){
//    ofstream outfile;
//    outfile.open(filename);
//    for (int vpos = 0; vpos != howmany; vpos++) {
//        printable(outfile, vpos, lmat, tmat);
//    }
//    outfile.close();
//        //    ofstream vectorfile;
//        //    vectorfile.open("vectors.txt");
//        //    for (int vpos = 1; vpos != 1000; vpos++) {
//        //        printable(vectorfile, vpos, leech, trans);
//        //    }
//        //    vectorfile << "ENDFRAME\n";
//        //    vectorfile.close();
//}

#define POWERDIVISOR 6000

Matrix24f nextmat(Matrix24f mat0, double power){
    power += (1/(POWERDIVISOR));
    return mat0.pow(power);
}

//int sapphireRGB = ((int)15) << 16 | ((int)82) << 8 | ((int)186);
//pcl::PointXYZ vect2point(Vector3d v){
//    pcl::PointXYZ point;
//    point.x = (float) v(0);
//    point.y = (float) v(1);
//    point.z = (float) v(2);
//    return point;
//}

#define POINTCLOUDSIZE 100000

pcl::PointCloud<pcl::PointXYZ>::Ptr computecloud(double power, Matrix24f lmat, Matrix24f tmat0){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ> * cloud;
//    cloud = new pcl::PointCloud<pcl::PointXYZ>;
//    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    Matrix24f tmat = tmat0.pow(power);
    // Fill in the cloud data
    cloud->width  = POINTCLOUDSIZE;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        Vector24f v0 = binvec2(i);
        v0 = lmat*v0;
        v0 = tmat*v0;
        Vector3d v1 = extract3d(v0);
        cloud->points[i].x = (float) v1(0);
        cloud->points[i].y = (float) v1(1);
        cloud->points[i].z = (float) v1(2);
    }
    return cloud;
}

void rendercloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename){
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (1000);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (0.025);
    gp3.setSearchRadius (8.0);
    
    // Set typical values for the parameters
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (100);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);

    gp3.setMu (4.0);
    gp3.setMaximumNearestNeighbors (3000);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/12);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);
    gp3.setConsistentVertexOrdering(false);

    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Additional vertex information
    // I believe that this is unneeded for simply exporting the vtk..
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();
    
    pcl::io::saveVTKFile (filename, triangles);
}

void doitall(int n, Matrix24f lmat, Matrix24f tmat0){
    cout << "power: " << n << endl;
    double power = (double) (n / (POWERDIVISOR));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = computecloud(power, lmat, tmat0);

    std::string nstr = std::to_string(n);
    std::string filename = "mesh_" + nstr + ".vtk";
    
    rendercloud(cloud, filename);
}

int main()
{
    Matrix24f leech;
    leech <<    8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 2, 2, 2, 0, 0, 0, 0, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0,
                2, 0, 2, 0, 2, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0,
                2, 0, 0, 2, 2, 2, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0,
                2, 2, 0, 0, 2, 0, 2, 0, 2, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0,
                0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 2, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0,
                -3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

    Matrix24f trans;
    trans <<    0, 0, 1, 0, 0, 0,-1, 1, 1, 0,-1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1,
                3, 1,-1,-3,-2,-2, 0,-3, 1, 2, 2,-1, 1, 0, 1, 0, 0,-1, 0,-1, 0,-1,-1, 0,
                -4, 0,-1, 7, 4, 3, 1, 4,-3,-3,-2, 0,-1,-1,-1,-1, 0, 0, 1,-1, 0,-1, 1,-1,
                0,-1, 0, 1, 0, 1,-1, 1, 0, 0, 0,-1, 1, 0, 1,-1, 0, 0, 0, 0, 0, 0, 0, 0,
                6, 0, 2,-6,-4,-2,-3,-2, 1, 2, 1,-1, 1, 1, 2, 1, 1,-1,-1, 0, 0, 1, 0, 1,
                2, 0,-1,-5,-3,-3,-1,-4, 3, 3, 1, 0, 2, 0, 1, 1, 0,-1, 0, 0, 0, 1,-1, 1,
                -10, 0, 0, 12, 8, 5, 4, 7,-3,-6,-3, 1,-3,-1,-3,-1, 0, 3, 1, 1, 1,-1, 1,-1,
                -1, 4, 1, 5, 4, 1, 3, 3,-2,-3,-2, 1,-1,-1, 0, 0, 1, 1, 1, 0, 0,-1, 1, 0,
                -1, 0, 1, 3, 2, 1,-1, 3,-1,-1,-2, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0,
                2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0,-1, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 1,
                -5, 3, 0, 11, 7, 4, 4, 6,-4,-5,-3, 0,-2,-2,-1,-2, 1, 1, 1,-1, 1,-2, 1,-1,
                3, 0, 1,-2,-1,-1,-1, 0, 0, 1,-1, 0, 1, 0, 1, 0, 0,-1, 0, 0, 0, 1, 0, 0,
                -4, 0,-1, 5, 3, 2, 2, 2,-1,-3, 0, 0,-2,-1,-2,-1, 0, 1, 0, 0, 0,-1, 0,-1,
                2,-1, 1,-2,-1,-1,-1, 0, 1, 1, 1,-1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0,-1, 0,
                2,-1, 1,-3,-1,-1,-1, 0,-1, 1,-1, 1,-1, 1, 0, 1,-1, 0, 0, 0, 0, 1, 1,-1,
                -2, 0,-1, 3, 1, 1, 1, 1, 0,-1, 0, 0, 0,-1, 0,-1, 0, 1, 0, 0, 0, 0, 0, 0,
                -11,-2,-1, 12, 7, 5, 3, 7,-2,-5,-3, 1,-2,-1,-3,-2, 0, 2, 1, 1, 1, 0, 0,-1,
                -2, 0, 1, 2, 2, 1, 0, 3,-1,-1,-2, 1,-1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
                -6, 1,-1, 7, 5, 2, 2, 4,-2,-2,-3, 1,-1,-1,-1,-1, 1, 0, 1, 0, 1, 0, 0, 0,
                -3, 1,-1, 3, 2, 1, 2, 0, 0,-1, 0, 0,-1,-1,-1,-1, 0, 0, 0, 0, 0,-1, 0, 0,
                2,-2,-2,-6,-5,-2,-2,-6, 3, 4, 3,-1, 2, 0, 0, 0,-1,-1,-1,-1,-1, 0,-1, 0,
                2,-1,-2,-4,-4,-1,-1,-5, 2, 2, 4,-2, 1, 0, 0, 0,-1, 0,-1,-2, 0,-1,-1, 0,
                -6, 1, 0, 8, 5, 4, 3, 3,-2,-4,-1, 0,-2,-1,-2,-1, 0, 2, 0, 0, 0,-2, 1,-1,
                14, 1, 2,-18,-11,-7,-4,-10, 4, 7, 4, 0, 3, 2, 3, 3,-1,-2,-2, 0,-1, 2,-1, 2;
    
//    Matrix3d m = Matrix3d::Random();
//    m = (m + Matrix3d::Constant(1.2)) * 50;
//    cout << "m =" << endl << m << endl;
//    Vector3d v(1,2,3);
//    cout << "m * v =" << endl << m * v << endl;
//    Vector3d vtest = transd3d(1234, leech, trans);
//    cout << vtest << endl;

//    double power = 0;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = makecloud(double power, Matrix24f leech, Matrix24f trans);
    
//    for (int frame = 0; frame < 18000; frame++){
//        doitall(frame,leech,trans);
//    }
    
//    doitall(1, leech, trans);
//    doitall(12, leech, trans);
//    doitall(123, leech, trans);
//    doitall(1234, leech, trans);
//    doitall(12345, leech, trans);
    doitall(123456, leech, trans);
    doitall(1234567, leech, trans);
    
    return 0;
}