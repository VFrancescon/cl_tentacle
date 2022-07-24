#include <iostream>
#include "DataTypes.hpp"

using namespace Eigen;

int main(int argc, char* argv[]);

struct DefaultValues{
    double dL = 10e-3;
    double d = 2e-3;
    double E = 100e3;
    double v = 0.43;
};

Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles);
void Z_P_Precalc(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, int jointNo);
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec, int jointEff);
MatrixXd MechWrench(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, int jointEff );
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices);
void RotateUm(Vector3d StartingOrientation, std::vector<Joint> &iJoints, int jointEff);
MatrixXd MagWrench(std::vector<Joint> &iJoints, int jointEff);
MatrixXd MagWrenchMap(Vector3d &Mag);