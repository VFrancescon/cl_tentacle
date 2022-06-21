#include <iostream>
#include <vector>
#include "DataTypes.hpp"

int main(void){
	
	PosOrientation iPos; //initialises p as (0,0,0).t

	Joint iJoint(0, iPos);

	std::cout << "Pos initially: " << *iJoint.p << "\n";
	std::cout << "Orientation intially:\n" << *iJoint.z << "\n";
	
	Vector3f vec3(10,10,10);
	Matrix3f mat3;

	mat3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;

	iPos.p += vec3;
	iPos.z = iPos.z + mat3;

	std::cout << "Pos after addition:\n" << *iJoint.p << "\n";
	std::cout << "Orientation after addition:\n" << *iJoint.z << "\n";
	
	return 0;
}
