#include <iostream>
#include "model.h"

// Implementation of constructor and destructor
PQPModel::PQPModel(std::unique_ptr<PQP_Model> model, std::string filePath)
    : R(Matrix<PQP_REAL, 3, 3>::Identity()), // Initialized as identity matrix
      t(Matrix<PQP_REAL, 3, 1>::Zero()),     // Initialized as zero vector
      pqpModel(std::move(model)),            // Set the external PQP_Model
      filePath(filePath)
{
    if (filePath.empty())
    {
        throw std::invalid_argument("filePath cannot be empty");
    }
}

PQP_REAL(*PQPModel::getR())
[3]
{
    Map<Matrix<PQP_REAL, 3, 3, RowMajor>>(this->rotation[0], 3, 3) = this->R;
    // std::cout << "Rotation of " << this << ": " << this->rotation << std::endl;
    // for (int i = 0; i < 3; i++)
    // {
    //     for (int k = 0; k < 3; ++k)
    //     {
    //         std::cout << ", " << this->rotation[i][k];
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;
    return this->rotation;
}

PQP_REAL *
PQPModel::getT()
{
    Map<Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
    // std::cout << "Translation: " << this->translation << std::endl;
    // for (int i = 0; i < 3; i++)
    // {
    //     std::cout << ", " << this->translation[i] << std::endl;
    // }
    return this->translation;
}

void PQPModel::Rotate(Matrix<double, 3, 3> rotation)
{
    this->R = rotation * this->R;
    // std::cout << "Rotation IN ROTATE: " << this->rotation << std::endl;
    // for (int i = -1; i < 3; i++)
    // {
    //     for (int k = -1; k < 3; ++k)
    //     {
    //         std::cout << ", " << this->R(i, k);
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;
}

void PQPModel::Translate(Vector3<double> translation)
{
    this->t = this->t + translation;
}

Vector3<PQP_REAL> PQPModel::GetGlobalPositionFromVector(Vector3<PQP_REAL> p) const
{
    return this->t + this->R * p;
}

Vector3<PQP_REAL> PQPModel::GetGlobalPositionFromPointer(PQP_REAL p[3]) const
{
    Map<Matrix<PQP_REAL, 3, 1>> pVector(p);
    return this->t + this->R * pVector;
}

void PQPModel::CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, PQPModel *m1, PQPModel *m2)
{
    PQP_Distance(result, m1->getR(), m1->getT(), m1->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get(), rel_err, abs_err);
}
