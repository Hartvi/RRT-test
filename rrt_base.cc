#include <flann/flann.hpp>
#include <vector>
#include <random>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include "rrt_base.h"

TrPQPEnvWrapper::TrPQPEnvWrapper() {}

/// @brief Initialize the environment with existing & loaded models
TrPQPEnvWrapper::TrPQPEnvWrapper(std::vector<std::shared_ptr<TrPQPModel>> models)
{
    this->pqp_models = models;
    this->node_wrapper = std::make_unique<RRTNodeWrapper>(pqp_models[0]->getT());
}

void TrPQPEnvWrapper::AddPQPModelFromPath(std::string filePath)
{
    this->pqp_models.push_back(std::make_shared<TrPQPModel>(filePath));
    if (this->pqp_models.size() == 1)
    {
        this->node_wrapper = std::make_unique<RRTNodeWrapper>(this->pqp_models[0]->getT());
    }
}

void TrPQPEnvWrapper::AddPQPModelExisting(std::shared_ptr<TrPQPModel> pqp_model)
{
    this->pqp_models.push_back(pqp_model);
    if (this->pqp_models.size() == 1)
    {
        this->node_wrapper = std::make_unique<RRTNodeWrapper>(this->pqp_models[0]->getT());
    }
}

// runtime functions

bool TrPQPEnvWrapper::CheckCollision()
{
    if (this->pqp_models.size() == 1)
    {
        throw std::runtime_error("CheckCollision: no collision can be made, only one object present");
    }

    PQP_CollideResult collision_result;

    for (size_t j = 1; j < this->pqp_models.size(); ++j)
    {
        this->pqp_models[0]->Collide(&collision_result, this->pqp_models[j].get());
        if (collision_result.Colliding())
        {
            return true;
        }
    }
    return false;
}

void TrPQPEnvWrapper::RRT(double world_bound, std::string target_path, size_t max_iters)
{
    std::ofstream f;
    f.open(target_path);

    f << "m" << 0 << "," << this->pqp_models[0].get()->filePath;
    for (size_t j = 1; j < this->pqp_models.size(); ++j)
    {
        f << ",m" << j << "," << this->pqp_models[j].get()->filePath;
    }
    f << std::endl;

    double *init_location = this->node_wrapper.get()->GetLocation(0);
    f << "init," << init_location[0] << "," << init_location[1] << "," << init_location[2] << std::endl;
    f << "goal," << this->goal[0] << "," << this->goal[1] << "," << this->goal[2] << std::endl;

    const size_t N = max_iters;

    // Initialize random engine and distribution
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // Fill the array with random numbers
    for (size_t i = 0; i < N; i++)
    {
        // make a cube with corners in (-world_bound, world_bound)
        double r3[3];
        r3[0] = (2 * dist(rng) - 1) * world_bound;
        r3[1] = (2 * dist(rng) - 1) * world_bound;
        r3[2] = (2 * dist(rng) - 1) * world_bound;

        // TODO: check collision. If not colliding, get nearest point, otherwise continue cycle
        Eigen::Vector3d target_vector = Eigen::Vector3d(r3);
        this->pqp_models[0].get()->SetTranslation(target_vector);

        if (this->CheckCollision())
        {
            // std::cout << "RANDOM POINT: collision at: " << target_vector << std::endl;
            f << "rand_only,"
              << r3[0]
              << ","
              << r3[1]
              << ","
              << r3[2] << std::endl;
            continue;
        }

        int nearest_point_index = this->node_wrapper->nearest(r3);
        Eigen::Vector3d nearest_point_vector = Eigen::Vector3d(this->node_wrapper->GetNodeLocation(nearest_point_index));
        double multiplying_factor = 1.0;

        Eigen::Vector3d closer_point = nearest_point_vector + (target_vector - nearest_point_vector).normalized() * multiplying_factor;
        this->pqp_models[0].get()->SetTranslation(closer_point);

        if (this->CheckCollision())
        {
            // std::cout << "RANDOM POINT: collision at: " << target_vector << std::endl;
            // std::cout << "CLOSE POINT: collision at: " << closer_point << std::endl;
            f << "orig," << nearest_point_vector[0] << "," << nearest_point_vector[1] << "," << nearest_point_vector[2] << std::endl;
            f << "close," << closer_point[0] << "," << closer_point[1] << "," << closer_point[2] << std::endl;
            f << "rand," << r3[0] << "," << r3[1] << "," << r3[2] << std::endl;
            continue;
        }
        // TODO: check collision of closer point, if not colliding, then add it to the tree

        this->node_wrapper.get()
            ->add_node(nearest_point_index, closer_point.data());
        // this->node_wrapper.get()->buildIndex();
        // std::cout << "Number of nodes: " << this->node_wrapper.get()->GetNumberOfNodes() << std::endl;
        f << "new," << closer_point[0] << "," << closer_point[1] << "," << closer_point[2] << ",";
        double *prevLocation = this->node_wrapper.get()->GetNodeLocation(nearest_point_index);
        f << "from," << prevLocation[0] << "," << prevLocation[1] << "," << prevLocation[2] << std::endl;
        // reached goal
        if ((this->goal - closer_point).norm() < multiplying_factor)
        {
            break;
        }
    }
    std::cout << "Number of nodes: " << this->node_wrapper.get()->GetNumberOfNodes() << std::endl;
    f.close();
    // TODO VISUALIZE IN BLENDER
}

void TrPQPEnvWrapper::SetGoal(const Eigen::Vector3d v)
{
    this->goal = v;
}
