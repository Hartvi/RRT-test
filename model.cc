#include "model.h"

// Implementation of constructor and destructor
PQPModel::PQPModel(PQP_Model *model)
    : R(Matrix<PQP_REAL, 3, 3>::Identity()), // Initialized as identity matrix
      t(Matrix<PQP_REAL, 3, 1>::Zero()),     // Initialized as zero vector
      pqpModel(model)                        // Set the external PQP_Model
{
}

PQP_REAL(*PQPModel::getR())
[3]
{
    Map<Matrix<PQP_REAL, 3, 3>>(this->rotation[0], 3, 3) = this->R;
}

PQP_REAL *
PQPModel::getT()
{
    Map<Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
}

void PQPModel::Rotate(Matrix<double, 3, 3> rotation)
{
    this->R = this->R * rotation;
}

void PQPModel::Translate(Vector3<double> translation)
{
    this->t = this->t + translation;
}

static void CheckDistance(PQP_DistanceResult result, PQP_REAL rel_err, PQP_REAL abs_err, PQPModel *m1, PQPModel *m2)
{
    PQP_REAL rel_err = 0.0;
    PQP_REAL abs_err = 0.0;
    PQP_DistanceResult res;
    PQP_Distance(&res, m1->getR(), m1->getT(), m1->pqpModel, m2->getR(), m2->getT(), m2->pqpModel, rel_err, abs_err);
}
