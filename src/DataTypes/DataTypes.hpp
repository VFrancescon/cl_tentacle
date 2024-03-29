/**
 * @file DataTypes.h
 * @author Vittorio Francescon (vittorio.francescon@gmail.com)
 * @brief Contains definitions for structs containing key elements of the rigid link model.
 * @version 0.1
 * @date 21-06-2022
 * 
 */
#ifndef TENTACLEDATATYPES
#define TENTACLEDATATYPES

#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

/**
 * @brief Struct containing position and orientation of joint i in the chain
 * 
 */
struct PosOrientation{
    int index; //!< position in the chain
    Vector3d p; //!< position in space GLOBAL
    Matrix3d z; //!< orientation in space GLOBAL

    void setPosition(Vector3d Point);
    void setOrientation(Matrix3d Orientation);


};

inline void PosOrientation::setPosition(Vector3d Point){
    this->p = Point;
}

inline void PosOrientation::setOrientation(Matrix3d Orientation){
    this->z = Orientation;
}


/**
 * @brief Struct containing joint position and orientations, pointers to adjecent joints and transform matrix
 * 
 */
struct Joint{
    int index; //!< Position in joint chain
    Joint * nextJoint; //!< Pointer to next joint (i+1)
    Joint * prevJoint; //!< Pointer to previous joint (i-1)

    Vector3d *p; //!< joint position. Points to PosOrientation struct
    Matrix3d *z; //!< joint orientation. Points to Posorientation struct

    Vector3d q; //!< Global angle of joint

    Matrix3d Rotation; //!< Rotation part of Transform matrix
    Vector3d pLocal; //!< Positional part of Transform matrix

    //magnetisation here
    Vector3d GlobMag;
    Vector3d LocMag;

    //Global field and gradients
    MatrixXd GlobGrad;
    Vector3d GlobField;

    //Local field and gradients
    MatrixXd LocGrad;
    Vector3d LocField;

    void assignPosOri(PosOrientation &PosOri_);
};

inline void Joint::assignPosOri(PosOrientation &PosOri_){
    this->p = &PosOri_.p;
    this->z = &PosOri_.z;
}


/**
 * @brief Struct containing position of each link and mechanical parameters
 * 
 */
struct Link{
    int index; //position in link chain

    Vector3d *Pos1; //!< Joints i and i+1 that make up start and end of Link i
    Vector3d *Pos2; //!< Joints i and i+1 that make up start and end of Link i

    double dL; //!< Link Length
    double d; //!< Link Diameter
    int E; //!< Young's modulus
    double v; //!< Poissant's ratio

    void assignPosOri(PosOrientation &PosOri1_, PosOrientation &PosOri2_);

};


inline void Link::assignPosOri(PosOrientation &PosOri1_, PosOrientation &PosOri2_){
    this->Pos1 = &PosOri1_.p;
    this->Pos2 = &PosOri2_.p;

}

#endif
