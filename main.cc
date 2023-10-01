#include <iostream>
#include <Eigen/Dense>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "load.h"
#include "model.h"

void check_collision(PQPModel *m1, PQPModel *m2)
{
    // rotation
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Rx2;
    Eigen::Matrix3d Ry2;
    Eigen::Matrix3d Rz2;
    Rx2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    Ry2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
    Rz2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d result = I * Rx2;
    Eigen::Vector3d t1(1.0, 2.0, 3.0); // Just an example, you can set whatever values you need.

    PQP_REAL rel_err = 0.0;
    PQP_REAL abs_err = 0.0;
    PQP_DistanceResult res;
    // m1->getR();
    std::cout << Rz2 << std::endl;
    std::cout << Rz2 * I << std::endl;
    // m1->Rotate(Rz2);
    m1->Translate(t1);
    m2->Translate(-t1);
    // m1->getR();
    PQPModel::CheckDistance(&res, rel_err, abs_err, m1, m2);

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

    PQPModel *model1 = new PQPModel(obj1);
    PQPModel *model2 = new PQPModel(obj2);
    check_collision(model1, model2);
    return 0;
}