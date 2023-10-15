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
#include <memory>
#include "model.h"

#ifndef RRT_BASE_H
#define RRT_BASE_H

class RRTNode
{
public:
    /// @brief Index of the node in the RRTNodeWrapper.nodes array
    int parent;

    /// @brief 3D coordinates of the node in C-space
    double location[3];

    RRTNode(int p, double x, double y, double z) : parent(p)
    {
        location[0] = x;
        location[1] = y;
        location[2] = z;
    }
};

class RRTNodeWrapper
{
public:
    // RRTNodeWrapper() {}
    RRTNodeWrapper(const double rootLocation[3])
    {
        nodes.emplace_back(-1, rootLocation[0], rootLocation[1], rootLocation[2]);

        // Build the FLANN index
        buildIndex();
    }

    ~RRTNodeWrapper()
    {
        // delete index;
        delete[] data.ptr();
    }

    void add_node(int p, double xyz[3])
    {
        nodes.emplace_back(p, xyz[0], xyz[1], xyz[2]);

        // have to build index to register the new node
        buildIndex();
    }

    void buildIndex()
    {
        this->refreshDataFromNodes();
        this->refreshIndex();
    }

    int nearest(double new_point[3])
    {
        flann::Matrix<double> query(new_point, 1, 3);     // Single row matrix for the new point
        flann::Matrix<int> indices(new int[1], 1, 1);     // Single row matrix for the index
        flann::Matrix<double> dists(new double[1], 1, 1); // Single row matrix for the distance

        // Search for the closest point. We're only interested in the nearest one.
        this->index.get()->knnSearch(query, indices, dists, 1, flann::SearchParams(128));

        int closestIndex = indices[0][0];

        delete[] indices.ptr();
        delete[] dists.ptr();

        return closestIndex; // Return the index of the nearest node
    }
    double *GetNodeLocation(int index)
    {
        return this->nodes[index].location;
    }

    size_t GetNumberOfNodes()
    {
        return this->nodes.size();
    }

    double *GetLocation(int index)
    {
        return this->nodes[index].location;
    }

private:
    std::vector<RRTNode> nodes;
    flann::Matrix<double> data;
    std::unique_ptr<flann::Index<flann::L2<double>>> index;

    void refreshDataFromNodes()
    {
        delete[] this->data.ptr();

        double *_data_mtx = new double[this->nodes.size() * 3];

        this->data = flann::Matrix<double>(_data_mtx, this->nodes.size(), 3);

        for (size_t i = 0; i < this->nodes.size(); ++i)
        {
            this->data[i][0] = this->nodes[i].location[0];
            this->data[i][1] = this->nodes[i].location[1];
            this->data[i][2] = this->nodes[i].location[2];
        }
    }

    void refreshIndex()
    {
        this->index = std::make_unique<flann::Index<flann::L2<double>>>(this->data, flann::KDTreeIndexParams(4));
        this->index.get()->buildIndex();
    }
};

class TrPQPEnvWrapper
{
public:
    /// @brief 1st element is the robot model, the rest is obstacles
    std::vector<std::shared_ptr<TrPQPModel>> pqp_models;

    /// @brief location coordinates are independent of the models and comprise te RRT tree
    std::unique_ptr<RRTNodeWrapper> node_wrapper;

    TrPQPEnvWrapper();

    /// @brief Initialize the environment with existing & loaded models
    TrPQPEnvWrapper(std::vector<std::shared_ptr<TrPQPModel>> models);

    void AddPQPModelFromPath(std::string filePath);

    void AddPQPModelExisting(std::shared_ptr<TrPQPModel> pqp_model);

    // Destructor will automatically delete the unique pointers.
    ~TrPQPEnvWrapper() = default;

    // runtime functions

    bool CheckCollision();

    void RRT(double world_bound, std::string target_path = "", size_t max_iters = 1000);

    void SetGoal(const Eigen::Vector3d v);

private:
    Eigen::Vector3d goal;
};

#endif