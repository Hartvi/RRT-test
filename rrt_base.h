#include <flann/flann.hpp>
#include <vector>

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
    RRTNodeWrapper(const double rootLocation[3])
    {
        nodes.emplace_back(-1, rootLocation[0], rootLocation[1], rootLocation[2]);

        // Build the FLANN index
        buildIndex();
    }

    ~RRTNodeWrapper()
    {
        delete index;
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
        data = flann::Matrix<double>(new double[this->nodes.size() * 3], nodes.size(), 3);

        for (size_t i = 0; i < nodes.size(); ++i)
        {
            data[i][0] = nodes[i].location[0];
            data[i][1] = nodes[i].location[1];
            data[i][2] = nodes[i].location[2];
        }

        index = new flann::Index<flann::L2<double>>(data, flann::KDTreeIndexParams(4));
        index->buildIndex();
    }

    int nearest(double new_point[3])
    {
        flann::Matrix<double> query(new_point, 1, 3);     // Single row matrix for the new point
        flann::Matrix<int> indices(new int[1], 1, 1);     // Single row matrix for the index
        flann::Matrix<double> dists(new double[1], 1, 1); // Single row matrix for the distance

        // Search for the closest point. We're only interested in the nearest one.
        index->knnSearch(query, indices, dists, 1, flann::SearchParams(128));

        int closestIndex = indices[0][0];

        delete[] indices.ptr();
        delete[] dists.ptr();

        return closestIndex; // Return the index of the nearest node
    }

private:
    std::vector<RRTNode> nodes;
    flann::Matrix<double> data;
    flann::Index<flann::L2<double>> *index;
};

class TrPQPEnvWrapper
{
public:
    // std::vector<PQPModel *> nodes;
    /// @brief 1st element is the robot model, the rest is obstacles
    std::vector<std::unique_ptr<TrPQPModel>> nodes;

    /// @brief location coordinates are independent of the models and comprise te RRT tree
    RRTNodeWrapper *node_wrapper;

    /// @brief Initialize the environment with existing & loaded models
    TrPQPEnvWrapper(std::vector<std::unique_ptr<TrPQPModel>> models) : nodes(std::move(models))
    {
        std::cout << "Model build state: " << nodes[0].get()->pqpModel.get()->build_state << std::endl;
        std::cout << "Initial position: " << nodes[0].get()->getT()[0] << ", " << nodes[0].get()->getT()[1] << ", " << nodes[0].get()->getT()[2] << std::endl;

        const double initial_position[3] = {nodes[0].get()->getT()[0], nodes[0].get()->getT()[1], nodes[0].get()->getT()[2]};

        this->node_wrapper = new RRTNodeWrapper(nodes[0].get()->getT());
    }
    ~TrPQPEnvWrapper() // Destructor will automatically delete the unique pointers.
    {
        delete node_wrapper;
    }
};
