/**
 * @file DataTypes.h
 * @author Vittorio Francescon (vittorio.francescon@gmail.com)
 * @brief Contains definitions for structs containing key elements of the rigid link model.
 * @version 0.1
 * @date 21-06-2022
 * 
 */

#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

/**
 * @brief Struct containing position and orientation of joint i in the chain
 * 
 */
struct PosOrientation{
    int index; //!< position in the chain
    Vector3f p; //!< position in space GLOBAL
    Matrix3f z; //!< orientation in space GLOBAL

    void setPosition(Vector3f Point);
    void setOrientation(Matrix3f Orientation);


};

void PosOrientation::setPosition(Vector3f Point){
    this->p = Point;
}

void PosOrientation::setOrientation(Matrix3f Orientation){
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

    Vector3f *p; //!< joint position. Points to PosOrientation struct
    Matrix3f *z; //!< joint orientation. Points to Posorientation struct

    Vector3f q; //!< Global angle of joint

    Matrix3f Rotation; //!< Rotation part of Transform matrix
    Vector3f pLocal; //!< Positional part of Transform matrix

    //global magnetisation here
    Vector3f GlobMag;

    void assignPosOri(PosOrientation &PosOri_);
};

void Joint::assignPosOri(PosOrientation &PosOri_){
    this->p = &PosOri_.p;
    this->z = &PosOri_.z;
}


/**
 * @brief Struct containing position of each link and mechanical parameters
 * 
 */
struct Link{
    int index; //position in link chain

    Vector3f *Pos1; //!< Joints i and i+1 that make up start and end of Link i
    Vector3f *Pos2; //!< Joints i and i+1 that make up start and end of Link i

    float dL; //!< Link Length
    float d; //!< Link Diameter
    int E; //!< Young's modulus
    float v; //!< Poissant's ratio

    void assignPosOri(PosOrientation &PosOri1_, PosOrientation &PosOri2_);

};


void Link::assignPosOri(PosOrientation &PosOri1_, PosOrientation &PosOri2_){
    this->Pos1 = &PosOri1_.p;
    this->Pos2 = &PosOri2_.p;

}

