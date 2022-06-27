/**
 * @file main.cpp
 * @author Vittorio Francescon (vittorio.francescon@gmail.com)
 * @brief Development file for the computation of Jacobians and wrenches
 * @version 0.5
 * @date 24-06-2022
 * 
 * 
 */

#include <iostream>
#include <vector>
#include "DataTypes.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

using namespace Eigen::placeholders;





// Matrix3d RotationXYZ(Matrix3d src, double th_x_r, double th_y_r, double th_z_r);
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles);
Matrix3d RotationZYX(Vector3d jointAngles);
void Z_P_Precalc(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, int jointNo);
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec, int jointEff);
MatrixXd MechWrench(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, int jointEff );
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices);




MatrixXd MagWrench(std::vector<Joint> &iJoints, int jointEff);
MatrixXd MagWrenchMap(Vector3d &Mag);

int main(void){
	const int maxJoints = 5;
	int jointNo = 3;
	int linkNo = jointNo - 1;
	int jointEff = linkNo;

	std::vector<PosOrientation> iPosVec(maxJoints); //initialises p as (0,0,0).t
	Vector3d a(3,2,1);
	Matrix3d b;
	b << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	
	for(int i = 0; i < jointNo; i++){
		iPosVec[i].setPosition(a);
		iPosVec[i].setOrientation(b);
	}

	std::vector<Joint> iJoints(maxJoints);

	for(int i = 0; i < jointNo; i++){
		iJoints[i].assignPosOri(iPosVec[i]);
	}

	//For loop to print the p and z members of each joint that has been instantiated
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

	//For loop to print the Pos1 and Pos2 of each link that has been instantiated
	// for(int i = 0; i < linkNo; i++) {
	// 	std::cout << "i: " << i << "\nPos1\n" << *iLinks[i].Pos1 << "\nPos2\n" << *iLinks[i].Pos2 << "\n\n";
	// }

	//assigning values to joint angles
	iJoints[0].q = Vector3d(0,10,0);
	iJoints[1].q = Vector3d(0,20,0);
	iJoints[2].q = Vector3d(0,0,0);
	// iJoints[3].q = Vector3d(0,00,0);


	//DIRECT KINEMATICS starts here
	//assigning initial values to Transform matrix stand-in
	iJoints[0].Rotation = Matrix3d::Identity();
	iJoints[0].pLocal = Vector3d::Zero();

	for(int i = 1; i < jointNo; i++){
		iJoints[i].Rotation = RotationZYX(iJoints[i-1].Rotation, iJoints[i-1].q);
		iJoints[i].pLocal = iJoints[i-1].pLocal + iJoints[i].Rotation * Vector3d(0,0, -iLinks[i-1].dL);
	}
	//for loop to verify the DirectKinematics Function in the matlab code
	// for(int i = 0; i < jointNo; i++){
	// 	std::cout << "-------------------------------------------";
	// 	std::cout << "\ni " << i << " Rotation bit:\n" << iJoints[i].Rotation << "\nPosition bit\n" << iJoints[i].pLocal << "\n";
	// }

	Z_P_Precalc(iPosVec, iJoints, jointNo);
	MatrixXd Jacobian = EvaluateJacobian(iPosVec, jointEff);
	Jacobian.transposeInPlace();
	//Verified transposed Jacobian
	// std::cout << "Full Transposed jacobian of size " << Jacobian.rows() << " by " << Jacobian.cols() << " is:\n" << Jacobian << "\n\n"; 
	
	MatrixXd MeWrench = MechWrench(iLinks, iJoints, jointEff);
	//Verifies the mech wrench
	// std::cout << "MWrench:\n" << MWrench << "\n"; 



	return 0;
}

MatrixXd MagWrench(std::vector<Joint> &iJoints, int jointEff){
	
	Vector3d Mag;
	MatrixXd AppliedU(6, jointEff);
	MatrixXd Wrench(6, jointEff);
	for(int i = 0; i < jointEff; i++){
		Mag = iJoints[i].Rotation * iJoints[i].LocMag;
		Wrench(all, i) = MagWrenchMap(Mag) * AppliedU;
		
	}

	return Wrench;
}

/**
 * @brief Creates a map from magnetisation to wrench (taken from Salmanipour & Diller 2018 - Eight-Degrees-of-Freedom Remote Actuation of Small Magnetic Mechanisms)
 * 
 * @param Mag 
 * @return MatrixXd 
 */
MatrixXd MagWrenchMap(Vector3d &Mag){
	MatrixXd forceMap(3, 5);
	Matrix3d torqueMap;


	forceMap << Mag(0), Mag(1), Mag(2),    0   ,   0   ,
				   0  , Mag(0),   0   ,  Mag(1), Mag(2),
			   -Mag(2),    0  , Mag(0), -Mag(2), Mag(1); 

	torqueMap << 0, -Mag(2), Mag(1),
				Mag(2), 0, -Mag(0),
				-Mag(1), Mag(0), 0;

	MatrixXd S(6 ,8);
	S = MatrixXd::Zero(6,8);
	S(seq(0,2), seq(0,4)) = forceMap;
	S(seq(3,5), seq(5,7)) = torqueMap;
	return S; 	

}

MatrixXd MechWrench(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, int jointEff ){
	MatrixXd deflections(3*jointEff,1);
	
	//stacks joint angles into a 3*N column for later computation
	for(int i = 0; i < jointEff; i++){
		deflections(seq(0+i*3, 2+(i*3)), all) = iJoints[i].q;
	}

	std::vector<Matrix3d> K_vec;
	
	//calculates the stiffness matrix per given joint
	for(int i = 0; i < jointEff; i++){
		double lRadius = iLinks[i].d / 2;
		double I = M_PI_4 * lRadius * lRadius * lRadius * lRadius;
		double G = iLinks[i].E / (2* (iLinks[i].v + 1) );
		double J = M_PI_2 * lRadius * lRadius * lRadius * lRadius;
		double Kb = iLinks[i].E*I/iLinks[i].dL;
		double Kt = G*J/iLinks[i].dL;
		Matrix3d K = Matrix3d::Zero();
		K(0,0) = Kb;
		K(1,1) = Kb;
		K(2,2) = Kt;
		K_vec.push_back (K);
	}
	
	//stacks all stiffness matrices in a diagonal matrix
	MatrixXd KDiagonal;
	KDiagonal = StackDiagonals(K_vec);

	MatrixXd StackedWrench(3*jointEff, 1);
	StackedWrench = -KDiagonal * deflections;

	MatrixXd MWrench = StackedWrench.reshaped(3, jointEff);

	return MWrench;
}


void RotateUm(Vector3d StartingOrientation, MatrixXd appliedField, std::vector<Vector3d> magnetisation, int jointEff){
	double AngleX, AngleY, AngleZ;
	AngleX = StartingOrientation[0] * M_PI / 180;
	AngleY = StartingOrientation[1] * M_PI / 180;
	AngleZ = StartingOrientation[2] * M_PI / 180;

	Matrix3d RotationMat = Matrix3d::Ones() * AngleAxisd(AngleY, Vector3d::UnitY())
		* AngleAxisd(AngleX, Vector3d::UnitX())
		* AngleAxisd(AngleZ, Vector3d::UnitZ());

	//fieldsGlob = appliedField(5:7)

	//gradientsGlob = appliedField(0:3) 
	//gradientsGlob = big block of stuff

	//for i < jointEfff
		//fieldLocal = RotationMat * fieldsGlob
		//gradientLocal = RotationMat * gradientsGlob * RotationMat'
		//MagnetisationLocal = RotationMat * MagnetisationGlobal

		//stack field and gradient stuff
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
		
		iPosVec[i].z(all,0) = iJoints[i].Rotation * Vector3d::UnitX();
		
		iPosVec[i].z(all,1) = AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) *  
							iJoints[i].Rotation * Vector3d::UnitY();
		
		iPosVec[i].z(all,2) = AngleAxisd( iJoints[i].q(1) * M_PI / 180, Vector3d::UnitY() ) * 
							AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) * 
							iJoints[i].Rotation * Vector3d::UnitZ();	
	}
}

/**
 * @brief Evaluates the jacobian using orientations and positions containing in a vector of points
 * 
 * @param iPosVec vector contains Positions and Orientations
 * @param jointEff Number of effective joints (discount end effector basically)
 * @return MatrixXd size (joint)*6 x (jointEff)*3 containing the full Jacobian computed
 */
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec, int jointEff){
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
	 * stacking in the Matrix algebra library we use is possible, but
	 * a pain, so filling is good enough, probably.
	 */

	Matrix3d Jp, Jo;
	MatrixXd Jacobian(jointEff*6, jointEff*3);
	for(int i = 0; i < jointEff; i++){
		//i goes vertically
		for(int k = 0; k < jointEff; k++){
			//k goes horizontally
			if( k > i ) {
				Jp = Matrix3d::Zero();
				Jo = Matrix3d::Zero();
			} else{
				Vector3d pDiff = iPosVec[i+1].p - iPosVec[k].p;
				std::vector<Vector3d> z1{iPosVec[k].z(all,0), iPosVec[k].z(all,1), iPosVec[k].z(all,2)};
				std::vector<Vector3d> ZcrossP{z1[0].cross(pDiff), z1[1].cross(pDiff), z1[2].cross(pDiff)};
				Jp << ZcrossP[0] , ZcrossP[1], ZcrossP[2];
				Jo << z1[0], z1[1], z1[2];	
			}
			MatrixXd Jn( Jp.rows() + Jo.rows(), Jp.cols());	
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
 * @return Matrix3d, Rotated matrix after being multiplied by angles jointAngles 
 */
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles){
	double AngleZ = jointAngles(2) * M_PI / 180;
	double AngleY = jointAngles(1) * M_PI / 180;
	double AngleX = jointAngles(0) * M_PI / 180;
	
	return src * AngleAxisd(AngleZ, Vector3d::UnitZ())
		* AngleAxisd(AngleY, Vector3d::UnitY())
		* AngleAxisd(AngleX, Vector3d::UnitX());
}

/**
 * @brief Creates a diagonal matrix by stacking 3x3 matrices contained in vector matrices
 * 
 * @param matrices vector containing n 3x3 matrices
 * @return MatrixXd - 3*n, 3*n Matrix containing the diagonal stack
 */
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices){
	MatrixXd diagonal(matrices.size()*3, matrices.size()*3);
	for(size_t i = 0; i < matrices.size(); i++){
		
		diagonal( seq(i*3, 2+i*3), seq(i*3, 2+i*3)) = matrices[i];
	
	}
	return diagonal;
}