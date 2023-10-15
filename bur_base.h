#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>

#ifndef BUR_BASE_H
#define BUR_BASE_H

using namespace Eigen;

class BurNode
{
public:
    /// @brief Index of the node in the array
    size_t parent_idx;

    /// @brief Node location in configuration space
    VectorXd q;

    /// @brief Node location in euclidean space
    Vector3d t;

    BurNode(size_t p, VectorXd q, Vector3d t)
        : parent_idx(p),
          q(q),
          t(t)
    {
    }
};

class BurNodeWrapper
{
public:
    using ForwardKinematics = std::function<Vector3d(const Eigen::VectorXd &)>;

    BurNodeWrapper(ForwardKinematics f)
    {
        this->forwardKinematics = f;
    }

    void InitRoot(const VectorXd q_location)
    {
        this->nodes.emplace_back(-1, q_location);
        // Build the FLANN index
        buildIndex();
    }

    ~BurNodeWrapper()
    {
        // delete index;
        delete[] this->data.ptr();
    }

    void add_node(size_t p, const VectorXd q_location)
    {
        nodes.emplace_back(p, q_location);

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

    VectorXd GetQ(size_t index)
    {
        return this->nodes[index].q;
    }

    Vector3d GetT(size_t index)
    {
        return this->nodes[index].t;
    }

    size_t GetNumberOfNodes()
    {
        return this->nodes.size();
    }

private:
    std::vector<BurNode> nodes;
    flann::Matrix<double> data;
    std::unique_ptr<flann::Index<flann::L2<double>>> index;

    ForwardKinematics forwardKinematics;

    void refreshDataFromNodes()
    {
        delete[] this->data.ptr();

        double *_data_mtx = new double[this->nodes.size() * 3];

        this->data = flann::Matrix<double>(_data_mtx, this->nodes.size(), 3);

        for (size_t i = 0; i < this->nodes.size(); ++i)
        {
            this->data[i][0] = this->nodes[i].t[0];
            this->data[i][1] = this->nodes[i].t[1];
            this->data[i][2] = this->nodes[i].t[2];
        }
    }

    void refreshIndex()
    {
        this->index = std::make_unique<flann::Index<flann::L2<double>>>(this->data, flann::KDTreeIndexParams(4));
        this->index->buildIndex();
    }
};

#endif