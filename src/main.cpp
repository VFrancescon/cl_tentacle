#include <iostream>
#include <vector>
#include "DataTypes.hpp"

int main(void){
	const int maxJoints = 5;
	int jointNo = 2;

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

	// for(auto i: iPosVec) std::cout << "Pos\n" << i.p << "\nOrientation\n" << i.z << "\n\n";

	std::vector<Joint> iJoints(maxJoints);

	for(int i = 0; i < jointNo; i++){
		iJoints[i].assignPosOri(iPosVec[i]);
	}

	//For to print the p and z members of each joint that has been instantiated
	for(int i = 0; i < jointNo; i++) {
		std::cout << "i: " << i << "\nPos\n" << *iJoints[i].p << "\nOrientation\n" << *iJoints[i].z << "\n\n";
	}


	std::vector<Link> iLinks(maxJoints);

	for(int i = 0; i < jointNo-1; i++){
		iLinks[i].assignPosOri(iPosVec[i], iPosVec[i+1]);
	}

	// for(int i = 0; i < jointNo; i++) {
	// 	std::cout << "i: " << i << "\nPos1\n" << *iLinks[i].Pos1 << "\nPos2\n" << *iLinks[i].Pos2 << "\n\n";
	// }


	return 0;
}
