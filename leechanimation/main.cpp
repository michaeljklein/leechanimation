//
//  main.cpp
//  leechanimation
//
//  Created by Michael Klein on 12/20/15.
//  Copyright © 2015 Michael Klein. All rights reserved.
//

// Utilizes Eigen 2.8.2 and Assimp 3.2

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
using namespace Eigen;
using namespace std;

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

void printable(ofstream& vectorz, int n, Matrix24f lmat, Matrix24f tmat){
    Vector24f v0 = binvec(n);
    v0 = lmat*v0;
    v0 = tmat*v0;
    Vector3d v1 = extract3d(v0);
    vectorz << std::fixed << std::setprecision(10) << (v1(0) + DIMOFFSET);
    vectorz << " ";
    vectorz << std::fixed << std::setprecision(10) << (v1(1) + DIMOFFSET);
    vectorz << " ";
    vectorz << std::fixed << std::setprecision(10) << (v1(2) + DIMOFFSET);
    vectorz << "\n";
}

void writevs(string filename, int howmany, Matrix24f lmat, Matrix24f tmat){
    ofstream outfile;
    outfile.open(filename);
    for (int vpos = 0; vpos != howmany; vpos++) {
        printable(outfile, vpos, lmat, tmat);
    }
    outfile.close();
        //    ofstream vectorfile;
        //    vectorfile.open("vectors.txt");
        //    for (int vpos = 1; vpos != 1000; vpos++) {
        //        printable(vectorfile, vpos, leech, trans);
        //    }
        //    vectorfile << "ENDFRAME\n";
        //    vectorfile.close();
}

#define POWERDIVISOR 6000

Matrix24f nextmat(Matrix24f mat0, double power){
    power += (1/(POWERDIVISOR));
    return mat0.pow(power);
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
    
    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;
    cout << "m =" << endl << m << endl;
    Vector3d v(1,2,3);
    cout << "m * v =" << endl << m * v << endl;
    Vector3d vtest = transd3d(1234, leech, trans);
    cout << vtest << endl;

    
    
    
//    ofstream vectorfile;
//    vectorfile.open("vectors.txt");
//    
//    for (int vpos = 1; vpos != 1000; vpos++) {
//        printable(vectorfile, vpos, leech, trans);
//    }
//    
//    vectorfile << "ENDFRAME\n";
//    
//    vectorfile.close();
}