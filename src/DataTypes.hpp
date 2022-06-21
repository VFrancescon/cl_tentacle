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

    PosOrientation();
    PosOrientation(Vector3f Point, Matrix3f Orientation);

    void setPosition(Vector3f Point);
    void setOrientation(Matrix3f Orientation);


};

/**
 * @brief default ructor for P/Z struct
 * 
 * Initialises z and p to zeros
 */
PosOrientation::PosOrientation(){
    this->z = Matrix3f::Zero();
    this->p = MatrixXf::Zero(3, 1);
    std::cout << "Empty constructor called!\n";
}


PosOrientation::PosOrientation(Vector3f Point_, Matrix3f Orientation_){
    this->p = Point_;
    this->z = Orientation_;
    std::cout << "Argumented constructor called!\n";
}

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
    Joint();
    Joint(int index_, PosOrientation &PosOri_);
    void assignPosOri(PosOrientation &PosOri_);
};

Joint::Joint(){
    std::cout << "Empty constructor\n";
    this->p = nullptr;
    this->z = nullptr;
}

Joint::Joint(int index_, PosOrientation &PosOri_){
    this->index = index_;
    this->p = &PosOri_.p;
    this->z = &PosOri_.z;

}

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

    Vector3f Pos1, Pos2; //!< Joints i and i+1 that make up start and end of Link i

    int dL; //!< Link Length
    int d; //!< Link Diameter
    int E; //!< Young's modulus
    int v; //!< Poissant's ratio

    Link();
    Link(int index);
};