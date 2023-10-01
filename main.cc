#include <iostream>
#include <Eigen/Dense>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "load.h"

void copy_to_PQP3x3(PQP_REAL real[3][3], const Eigen::Matrix3<double> &m)
{
    // Ensure the matrix is 3x3
    if (m.rows() != 3 || m.cols() != 3)
    {
        throw std::invalid_argument("The provided matrix is not 3x3.");
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            real[i][j] = m(i, j);
        }
    }
}

void copy_to_PQP3(PQP_REAL real[3], const Eigen::Vector3<double> &m)
{
    std::cout << "rows: " << m.rows() << "  cols: " << m.cols() << std::endl;
    // // Ensure the matrix is 3x3
    // if (m.rows() != 3)
    // {
    //     throw std::invalid_argument("The provided matrix is not 3x3.");
    // }

    for (int i = 0; i < 3; ++i)
    {
        real[i] = m(i);
    }
}

void check_collision(PQP_Model *m1, PQP_Model *m2)
{
    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
    // PQP_REAL M1[3][3], M2[3][3], M3[3][3];

    // rotation
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Rx2;
    Eigen::Matrix3d Ry2;
    Eigen::Matrix3d Rz2;
    Rx2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    Ry2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
    Rz2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d result = I * Rx2;
    PQP_REAL real[3][3];
    copy_to_PQP3x3(real, result);

    // translation
    Eigen::Vector3d t(1.0, 2.0, 3.0); // Just an example, you can set whatever values you need.
    PQP_REAL real_vec[3];
    copy_to_PQP3(real_vec, t);

    T1[0] = -1;
    T1[1] = 0.0;
    T1[2] = 0.0;

    T2[0] = 1;
    T2[1] = 0.0;
    T2[2] = 0.0;

    for (int i = 0; i < 3; i++)
    {
        R1[i][i] = 1;
        R2[i][i] = 1;
    }

    // int mode;
    // double beginx, beginy;
    // double dis = 10.0, azim = 0.0, elev = 0.0;
    // double ddis = 0.0, dazim = 0.0, delev = 0.0;
    // double rot1 = 0.0, rot2 = 0.0, rot3 = 0.0;

    // MRotX(M1, rot1);
    // MRotY(M2, rot2);
    // MxM(M3, M1, M2);
    // MRotZ(M1, rot3);
    // MxM(R1, M3, M1);

    // MRotX(M1, rot3);
    // MRotY(M2, rot1);
    // MxM(M3, M1, M2);
    // MRotZ(M1, rot2);
    // MxM(R2, M3, M1);

    PQP_REAL rel_err = 0.0;
    PQP_REAL abs_err = 0.0;
    PQP_DistanceResult res;
    PQP_Distance(&res, R1, T1, m1, R2, T2, m2, rel_err, abs_err);
    std::cout << "Result: p1 ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << res.p1[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "Result: p2 ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << res.p2[i] << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "argc: ";
    std::cout << argc;
    std::cout << "\n";

    if (argc < 3)
    {
        std::cout << "Please enter two paths to an obj model to load.\n";
        return 1;
    }

    // lod obj then create pqp object

    tinyobj::ObjReader reader;
    PQP_Model *obj1 = new PQP_Model();
    PQP_Model *obj2 = new PQP_Model();

    pqploader::read_file(argv[1], reader);

    if (pqploader::tiny_OBJ_to_PQP_model(reader, obj1))
    {
        std::cout << "obj1 buildstate: " << obj1->build_state << std::endl;
        std::cout << "obj2 buildstate: " << obj2->build_state << std::endl;
    }

    pqploader::read_file(argv[2], reader);

    if (pqploader::tiny_OBJ_to_PQP_model(reader, obj2))
    {
        std::cout << "obj1 buildstate: " << obj1->build_state << std::endl;
        std::cout << "obj2 buildstate: " << obj2->build_state << std::endl;
    }

    check_collision(obj1, obj2);
    return 0;
}