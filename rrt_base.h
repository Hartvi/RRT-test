#include <flann/flann.hpp>
#include <vector>
#include <random>
#include <fstream>

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

    TrPQPEnvWrapper() {}

    /// @brief Initialize the environment with existing & loaded models
    TrPQPEnvWrapper(std::vector<std::shared_ptr<TrPQPModel>> models)
    {
        this->pqp_models = models;
        this->node_wrapper = std::make_unique<RRTNodeWrapper>(pqp_models[0]->getT());
    }

    void AddPQPModelFromPath(std::string filePath)
    {
        this->pqp_models.push_back(std::make_shared<TrPQPModel>(filePath));
        if (this->pqp_models.size() == 1)
        {
            this->node_wrapper = std::make_unique<RRTNodeWrapper>(this->pqp_models[0]->getT());
        }
    }

    void AddPQPModelExisting(std::shared_ptr<TrPQPModel> pqp_model)
    {
        this->pqp_models.push_back(pqp_model);
        if (this->pqp_models.size() == 1)
        {
            this->node_wrapper = std::make_unique<RRTNodeWrapper>(this->pqp_models[0]->getT());
        }
    }

    // Destructor will automatically delete the unique pointers.
    ~TrPQPEnvWrapper() = default;

    // runtime functions

    bool CheckCollision()
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

    void RRT(double world_bound, std::string target_path = "", size_t max_iters = 1000)
    {
        std::ofstream f;
        f.open(target_path);

        f << "m" << 0 << "," << this->pqp_models[0].get()->filePath;
        for (int j = 1; j < this->pqp_models.size(); ++j)
        {
            f << ",m" << j << "," << this->pqp_models[j].get()->filePath;
        }
        f << std::endl;

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
            Vector3d target_vector = Vector3d(r3);
            this->pqp_models[0].get()->SetTranslation(target_vector);

            if (this->CheckCollision())
            {
                std::cout << "RANDOM POINT: collision at: " << target_vector << std::endl;
                f << "rand_only,"
                  << r3[0]
                  << ","
                  << r3[1]
                  << ","
                  << r3[2] << std::endl;
                continue;
            }

            int nearest_point_index = this->node_wrapper->nearest(r3);
            Vector3d nearest_point_vector = Vector3d(this->node_wrapper->GetNodeLocation(nearest_point_index));
            double multiplying_factor = 0.1;

            Vector3d closer_point = nearest_point_vector + multiplying_factor * (target_vector - nearest_point_vector);
            this->pqp_models[0].get()->SetTranslation(closer_point);

            if (this->CheckCollision())
            {
                std::cout << "RANDOM POINT: collision at: " << target_vector << std::endl;
                std::cout << "CLOSE POINT: collision at: " << closer_point << std::endl;
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
            f << "no_col," << closer_point[0] << "," << closer_point[1] << "," << closer_point[2] << std::endl;
        }
        std::cout << "Number of nodes: " << this->node_wrapper.get()->GetNumberOfNodes() << std::endl;
        f.close();
        // TODO VISUALIZE IN BLENDER
    }

    void SetGoal(const Vector3d v)
    {
        this->goal = v;
    }

private:
    Vector3d goal;
};
