Start of the repo


# Dependecies

[Matrices: eigen](https://eigen.tuxfamily.org/dox/index.html)

# Datatype prototypes
```cpp

struct{
    const int idx; //position in joint chain
    const joint * next_joint (idx+1);
    const joint * prev_joint (idx-1);
    vec3i pi (px, py, pz).t;

    M3x3 zi( zix ,ziy, ziz); //all components are 3x1 column vectors

    vec3i qi(thix, thiy, thiz); //joint angles in 3d

} joint;

```