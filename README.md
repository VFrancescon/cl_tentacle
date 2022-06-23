# Header

## Dependecies

* [Matrices: eigen](https://eigen.tuxfamily.org/dox/index.html)

* [Optimisation: Nlopt](https://nlopt.readthedocs.io/en/latest/)

* [Plotting: MatplotLib](https://matplotlib-cpp.readthedocs.io/en/latest/)

## Datatype prototypes

### Joints Struct

```cpp
struct{
    const int idx; //position in joint chain
    const joint * next_joint (idx+1);
    const joint * prev_joint (idx-1);
    vec3 * pIdx = PosOrientation.pIdx;
    Mat3x3 *zIdx = PosOrientation.zIdx;

    vec3 qIdx (th_Idxx, th_Idxy, th_Idxz); //joint angles in 3d
    
    //stand-in for 4x4 Tranformation Matrix.
    //compute only if i > 0
    Mat3x3 RIdx ( vec3 rx, vec3 ry, vec rz );
    vec3 pLocalIdx = prev_joint.pLocalIdx + RIdx * link[idx].dL;


    //local magnetisation

    global_mag[i] = joint[i].mag * joint[i].RIdx;
    

} joint;

```

### Link Struct

```cpp

struct{
    const int idx; //position in link chain
    
    vec3 * pos1 = joint(idx).pIdx;

    vec3 * pos2 = joint(idx+1).pIdx;

    int dL = const; //link length
    int d = const; //link diameter
    int E = const; //young's modulus
    int v = const; //poissant's ratio

} link;

```

### Position

```cpp
struct{
    const int idx;
    vec3 pIdx (px, py, pz).t; //.t means transposed
    M3x3 zIdx ( z_Idxx ,z_Idxy, z_Idxz); //all components are 3x1 column vectors

} Pos_Orientation;

```

## Eigen notation

### Matrix Stacking

```cpp

Matrix3f Jp, Jp;
Vector3f Vec3;

Jp << 1, 2, 3, 4, 5, 6,7 ,8, 9;

Jo << 10, 20, 30, 40, 50, 60, 70, 80, 90;

//vertical

MatrixXf J(Jp.rows() + Jo.rows(), Jo.cols());
J << Jp, 
    Jo;

/*
Jp stacked vertically with Jo
1  2  3
2  5  6
3  8  9
10 20 30
40 50 60
70 80 90
*/

//horizontal
J(Jp.rows(), Jp.rows() + Jo.cols());
J << Jp, Jo;

/*
Jp stacked vertically with Jo
1  2  3 10 20 30
2  5  6 40 50 60
3  8  9 70 80 90

*/

```

### Slicing

```cpp

using namespace Eigen::placeholders;

Matrix3f Mat;
Vector3f Vec3

//select column i

Vec3 = Mat(all, i); 

//select row j
Vec3 = Mat(j, all);

```
