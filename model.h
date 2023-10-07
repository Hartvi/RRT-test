#include <Eigen/Dense>
#include <memory>
#include "PQP.h"

using namespace Eigen;

typedef Matrix<PQP_REAL, Dynamic, Dynamic, RowMajor> RowMatrixXi;

class PQPModel
{
public:
    // Constructors & Destructors
    PQPModel(std::unique_ptr<PQP_Model> model = nullptr, std::string filePath = "");
    ~PQPModel() = default;

    // Public member functions (e.g., setters and getters) can be added as needed

    // Public member variables
    Eigen::Matrix<PQP_REAL, 3, 3> R;     // Rotation matrix
    Eigen::Matrix<PQP_REAL, 3, 1> t;     // Translation vector
    std::unique_ptr<PQP_Model> pqpModel; // Pointer to PQP_Model which is collidable
    std::string filePath;

    PQP_REAL(*getR())
    [3];
    PQP_REAL *getT();
    void Rotate(Matrix<double, 3, 3> rotation);
    void Translate(Vector3<double> translation);

    Vector3<PQP_REAL> GetGlobalPositionFromVector(Vector3<PQP_REAL> p) const;
    Vector3<PQP_REAL> GetGlobalPositionFromPointer(PQP_REAL p[3]) const;
    // PQP_REAL rel_err = 0.0;
    // PQP_REAL abs_err = 0.0;
    // PQP_DistanceResult res;
    static void CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, PQPModel *m1, PQPModel *m2);

private:
    PQP_REAL rotation[3][3];
    PQP_REAL translation[3];
};
