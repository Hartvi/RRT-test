#include <sstream>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "load.h"
#include "model.h"
#include <flann/flann.hpp>
// #include <flann/io/hdf5.h>
#include "rrt_base.h"

// flann::flann_checks_t f;

using namespace std;

template <typename T>
std::string join(int numargs, T *args, const std::string &delimiter = ",")
{
    std::ostringstream oss;
    for (int i = 0; i < numargs; ++i)
    {
        if (i != 0)
        {
            oss << delimiter;
        }
        oss << args[i];
    }
    return oss.str();
}

template <typename Iterable>
std::string join(const Iterable &items, const std::string &delimiter = ",")
{
    std::ostringstream oss;
    auto it = items.begin();
    if (it != items.end())
    {
        oss << *it;
        ++it;
    }
    while (it != items.end())
    {
        oss << delimiter << *it;
        ++it;
    }
    return oss.str();
}

string print_state(TrPQPModel *m1, TrPQPModel *m2)
{
    PQP_REAL rel_err = 0.0;
    PQP_REAL abs_err = 0.0;
    PQP_DistanceResult res;
    ostringstream oss;

    TrPQPModel::CheckDistance(&res, rel_err, abs_err, m1, m2);

    oss << "p1,";
    oss << join(3, res.p1);
    oss << std::endl;

    oss << "R1,";
    oss << join(m1->R.eulerAngles(0, 1, 2));
    oss << std::endl;

    oss << "t1,";
    oss << join(m1->GetGlobalPositionFromPointer(res.p1));
    oss << std::endl;

    oss << "p2,";
    oss << join(3, res.p2);
    oss << std::endl;

    oss << "R2,";
    oss << join(m2->R.eulerAngles(0, 1, 2));
    oss << std::endl;

    oss << "t2,";
    oss << join(m2->GetGlobalPositionFromPointer(res.p2));
    oss << std::endl;
    return oss.str();
}

void loop_rotation(TrPQPModel *m1, TrPQPModel *m2, string outfile)
{
    // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rx2;
    Eigen::Matrix3d Ry2;
    Eigen::Matrix3d Rz2;
    int steps = 20;
    Rx2 = Eigen::AngleAxisd(M_PI / steps, Eigen::Vector3d::UnitX());
    Ry2 = Eigen::AngleAxisd(M_PI / steps, Eigen::Vector3d::UnitY());
    Rz2 = Eigen::AngleAxisd(M_PI / steps, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d t1(0.9, 0.0, 0.0);
    Eigen::Vector3d t2(-0.9, 0.0, 0.0);

    // PQP_REAL rel_err = 0.0;
    // PQP_REAL abs_err = 0.0;
    // PQP_DistanceResult res;

    // begin animation
    m1->Translate(t1);
    m2->Translate(t2);

    ofstream myfile;
    myfile.open(outfile);
    myfile << "f1," << m1->filePath << std::endl;
    myfile << "f2," << m2->filePath << std::endl;

    for (int i = 0; i < steps; i++)
    {
        myfile << print_state(m1, m2);
        m1->Rotate(Rx2);
    }
    for (int i = 0; i < steps; i++)
    {
        myfile << print_state(m1, m2);
        m1->Rotate(Ry2);
    }
    for (int i = 0; i < steps; i++)
    {
        myfile << print_state(m1, m2);
        m1->Rotate(Rz2);
    }
    myfile.close();
}

void check_collision(TrPQPModel *m1, TrPQPModel *m2)
{
    // rotation
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Rx2;
    Eigen::Matrix3d Ry2;
    Eigen::Matrix3d Rz2;
    Rx2 = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
    Ry2 = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY());
    Rz2 = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d result = I * Rx2;
    Eigen::Vector3d t1(2.0, 0.0, 0.0); // Just an example, you can set whatever values you need.

    PQP_REAL rel_err = 0.0;
    PQP_REAL abs_err = 0.0;
    PQP_DistanceResult res;
    // m1->getR();
    std::cout << Rx2 << std::endl;
    std::cout << Rx2 * I << std::endl;
    m1->Rotate(Ry2.transpose());
    m1->Translate(t1);
    m2->Translate(-t1);
    // m1->getR();
    TrPQPModel::CheckDistance(&res, rel_err, abs_err, m1, m2);

    std::cout << "Result: p1 ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << res.p1[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << m1->GetGlobalPositionFromPointer(res.p1) << std::endl;

    std::cout << std::endl;
    std::cout << "Result: p2 ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << res.p2[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << m2->GetGlobalPositionFromPointer(res.p2) << std::endl;
    std::cout << std::endl;
}

void test_RRT_tree()
{
}

int main(int argc, char **argv)
{
    std::cout << "argc: ";
    std::cout << argc;
    std::cout << "\n";

    if (argc < 4)
    {
        std::cout << "Please enter two paths to an obj model to load and an output text file path." << std::endl;
        return 1;
    }

    auto model1 = std::make_unique<TrPQPModel>(argv[1]);
    auto model2 = std::make_unique<TrPQPModel>(argv[2]);

    // check_collision(model1, model2);
    loop_rotation(model1.get(), model2.get(), argv[3]);

    std::vector<std::unique_ptr<TrPQPModel>> models;
    models.push_back(std::move(model1));
    models.push_back(std::move(model2));

    auto a = std::make_unique<TrPQPEnvWrapper>(std::move(models));

    // nearest point test
    double d3[3] = {1.0, 1.0, 1.0};
    int nearest_point_idx = a->node_wrapper->nearest(d3);

    std::cout << "a->nodes: " << a->nodes.size() << std::endl;
    std::cout << "nearest point: " << nearest_point_idx << std::endl;
    a->node_wrapper->add_node(nearest_point_idx, d3);

    nearest_point_idx = a->node_wrapper->nearest(d3);
    std::cout << "a->nodes: " << a->nodes.size() << std::endl;
    std::cout << "nearest point: " << nearest_point_idx << std::endl;

    return 0;
}