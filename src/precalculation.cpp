#include "precalculation.hpp"

int main(int argc, char* argv[]){
    
    //assing number of joints
    int jointNo;
    if(argc > 1) jointNo = argc;
    else jointNo = 4;

    //assign joint angles
    std::vector<PosOrientation> iPosVec(jointNo); //initialises p as (0,0,0).t
	std::vector<Joint> iJoints(jointNo);



    iJoints[0].q = Vector3d(0,10,0);
    iJoints[1].q = Vector3d(0,10,0);
    iJoints[2].q = Vector3d(0,10,0);
    iJoints[3].q = Vector3d(0,10,0);


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


void RotateUm(Vector3d StartingOrientation, std::vector<Joint> &iJoints, int jointEff){
	double AngleX, AngleY, AngleZ;
	AngleX = StartingOrientation[0] * M_PI / 180;
	AngleY = StartingOrientation[1] * M_PI / 180;
	AngleZ = StartingOrientation[2] * M_PI / 180;

	Matrix3d RotationMat = Matrix3d::Ones() * AngleAxisd(AngleY, Vector3d::UnitY())
		* AngleAxisd(AngleX, Vector3d::UnitX())
		* AngleAxisd(AngleZ, Vector3d::UnitZ());

	for(int i = 0; i < jointEff; i++){
		Matrix3d dU_Global;
		dU_Global <<	iJoints[i].GlobGrad(0), iJoints[i].GlobGrad(1), iJoints[i].GlobGrad(2),
					  	iJoints[i].GlobGrad(1), iJoints[i].GlobGrad(3), iJoints[i].GlobGrad(4),
						iJoints[i].GlobGrad(2), iJoints[i].GlobGrad(4), -iJoints[i].GlobGrad(0) - iJoints[i].GlobGrad(3);
		Matrix3d dU_Local; 
		dU_Local = RotationMat * dU_Global * RotationMat.transpose();
		MatrixXd LocGrad(5,1);
		LocGrad << 	dU_Local(0,0), dU_Local(0,1), dU_Local(0,2),
					dU_Local(1,1), dU_Local(1,2);
		iJoints[i].LocGrad = LocGrad;
		iJoints[i].LocField = RotationMat * iJoints[i].GlobField;
		iJoints[i].LocMag = RotationMat * iJoints[i].GlobMag;
	}

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