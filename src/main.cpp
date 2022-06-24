#include <iostream>
#include <vector>
#include "DataTypes.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

using namespace Eigen::placeholders;


// Matrix3f RotationXYZ(Matrix3f src, float th_x_r, float th_y_r, float th_z_r);
Matrix3f RotationZYX(Matrix3f src, Vector3f jointAngles);
Matrix3f RotationZYX(Vector3f jointAngles);
void Z_P_Precalc(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, int jointNo);
MatrixXf EvaluateJacobian(std::vector<PosOrientation> &iPosVec, int jointEff);


int main(void){
	const int maxJoints = 5;
	int jointNo = 4;
	int linkNo = jointNo - 1;
	int jointEff = linkNo;
	// PosOrientation Pos1, Pos2, Pos3, Pos4, Pos5;
	// Joint Joint1, Joint2, Joint3, Joint4, Joint5;
	// std::vector<PosOrientation> Pos(10);

	std::vector<PosOrientation> iPosVec(maxJoints); //initialises p as (0,0,0).t
	Vector3f a(3,2,1);
	Matrix3f b;
	b << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	
	for(int i = 0; i < jointNo; i++){
		iPosVec[i].setPosition(a);
		iPosVec[i].setOrientation(b);
	}

	std::vector<Joint> iJoints(maxJoints);

	for(int i = 0; i < jointNo; i++){
		iJoints[i].assignPosOri(iPosVec[i]);
	}

	//For to print the p and z members of each joint that has been instantiated
	// for(int i = 0; i < jointNo; i++) {
	// 	std::cout << "i: " << i << "\nPos\n" << *iJoints[i].p << "\nOrientation\n" << *iJoints[i].z << "\n\n";
	// }

	std::vector<Link> iLinks(maxJoints);
	for(int i = 0; i < linkNo; i++){
		iLinks[i].assignPosOri(iPosVec[i], iPosVec[i+1]);

		//move to a function at some point, for now they are uniform
		iLinks[i].dL = 10e-3;
		iLinks[i].d = 2e-3;
		iLinks[i].E = 100e3;
		iLinks[i].v = 0.43;
	}

	//For to print the Pos1 and Pos2 of each link that has been instantiated
	// for(int i = 0; i < linkNo; i++) {
	// 	std::cout << "i: " << i << "\nPos1\n" << *iLinks[i].Pos1 << "\nPos2\n" << *iLinks[i].Pos2 << "\n\n";
	// }

	//assigning values to joint angles
	iJoints[0].q = Vector3f(0,10,0);
	iJoints[1].q = Vector3f(0,20,0);
	iJoints[2].q = Vector3f(0,00,0);
	// iJoints[3].q = Vector3f(0,00,0);


	//DIRECT KINEMATICS starts here
	//assigning initial values to Transform matrix stand-in
	iJoints[0].Rotation = Matrix3f::Identity();
	iJoints[0].pLocal = Vector3f::Zero();

	for(int i = 1; i < jointNo; i++){
		iJoints[i].Rotation = RotationZYX(iJoints[i-1].Rotation, iJoints[i-1].q);
		iJoints[i].pLocal = iJoints[i-1].pLocal + iJoints[i].Rotation * Vector3f(0,0, -iLinks[i-1].dL);
	}
	//for loop to verify the DirectKinematics Function in the matlab code
	// for(int i = 0; i < jointNo; i++){
	// 	std::cout << "-------------------------------------------";
	// 	std::cout << "\ni " << i << " Rotation bit:\n" << iJoints[i].Rotation << "\nPosition bit\n" << iJoints[i].pLocal << "\n";
	// }

	Z_P_Precalc(iPosVec, iJoints, jointNo);
	MatrixXf Jacobian = EvaluateJacobian(iPosVec, jointEff);
	std::cout << "Full jacobian of size " << Jacobian.rows() << " by " << Jacobian.cols() << " is:\n" << Jacobian << "\n\n"; 
	

	return 0;
}

/**
 * @brief Precalculates axis unit vectors and positional vectors for Jacobian computation
 * 
 * @param iPosVec vector containing joint positions and axis orientation
 * @param iJoints vector containing the joint's Transform matrix and pointers to iPosVec
 * @param jointNo number of joints (+1 from effective joint as we count end effector)
 */
void Z_P_Precalc(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, int jointNo){
	for(int i = 0; i < jointNo; i++){
		
		iPosVec[i].p = iJoints[i].pLocal;
		
		iPosVec[i].z(all,0) = iJoints[i].Rotation * Vector3f::UnitX();
		
		iPosVec[i].z(all,1) = AngleAxisf( iJoints[i].q(0) * M_PI / 180, Vector3f::UnitX() ) *  
							iJoints[i].Rotation * Vector3f::UnitY();
		
		iPosVec[i].z(all,2) = AngleAxisf( iJoints[i].q(1) * M_PI / 180, Vector3f::UnitY() ) * 
							AngleAxisf( iJoints[i].q(0) * M_PI / 180, Vector3f::UnitX() ) * 
							iJoints[i].Rotation * Vector3f::UnitZ();	
	}
}

/**
 * @brief Evaluates the jacobian using orientations and positions containing in a vector of points
 * 
 * @param iPosVec vector contains Positions and Orientations
 * @param jointEff Number of effective joints (discount end effector basically)
 * @return MatrixXf size (joint)*6 x (jointEff)*3 containing the full Jacobian computed
 */
MatrixXf EvaluateJacobian(std::vector<PosOrientation> &iPosVec, int jointEff){
	/**
	 * @note
	 * 
	 * Given J = [ 	J00 J01
	 * 				J10 J11 	]
	 * Where J_xy = [Jp_xy
	 * 				Jo_xy]
	 * 
	 * In the loops below, 	i tracks y
	 * 						k tracks x
	 * 
	 * Also the 'stacking' of the full jacobian is actually done by 
	 * initialising an empty Mat of the correct size and filling in the blocks
	 * stacking in the Matrix algebra library we use is possible
	 * just a pain, so filling is good enough, probably.
	 */

	Matrix3f Jp, Jo;
	MatrixXf Jacobian(jointEff*6, jointEff*3);
	for(int i = 0; i < jointEff; i++){
		//i goes vertically
		for(int k = 0; k < jointEff; k++){
			//k goes horizontally
			if( k > i ) {
				Jp = Matrix3f::Zero();
				Jo = Matrix3f::Zero();
			} else{
				Vector3f pDiff = iPosVec[i+1].p - iPosVec[k].p;
				std::vector<Vector3f> z1{iPosVec[k].z(all,0), iPosVec[k].z(all,1), iPosVec[k].z(all,2)};
				std::vector<Vector3f> ZcrossP{z1[0].cross(pDiff), z1[1].cross(pDiff), z1[2].cross(pDiff)};
				Jp << ZcrossP[0] , ZcrossP[1], ZcrossP[2];
				Jo << z1[0], z1[1], z1[2];	
			}
			MatrixXf Jn( Jp.rows() + Jo.rows(), Jp.cols());	
			Jn << Jp, 
				Jo;
			Jacobian(seq(0+i*6, 5+i*6), seq(0+k*3,2+k*3)) = Jn;
		}
	}
	return Jacobian;
}


/**
 * @brief Rotates matrix src by angles in vector jointAngles in the ZYX order.
 * 
 * @param src matrix containing original position
 * @param jointAngles column vector containing rotations. (X,Y,Z) 
 * @return Matrix3f, Rotated matrix after being multiplied by angles jointAngles 
 */
Matrix3f RotationZYX(Matrix3f src, Vector3f jointAngles){
	float AngleZ = jointAngles(2) * M_PI / 180;
	float AngleY = jointAngles(1) * M_PI / 180;
	float AngleX = jointAngles(0) * M_PI / 180;
	
	return src * AngleAxisf(AngleZ, Vector3f::UnitZ())
		* AngleAxisf(AngleY, Vector3f::UnitY())
		* AngleAxisf(AngleX, Vector3f::UnitX());
}