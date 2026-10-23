# gdk-collisions

3D collision detection and resolution library for games.

## Getting Started
Install cmake 3.21 or greater, and a c++20 compatible compiler 
```
# initialize the tree
git submodule update --init --recursive

# generate the script to build the library on linux using gcc
cmake --preset linux-gcc

# build the library
cmake --build --preset linux-gcc

# run the test suite
ctest --preset linux-gcc
```
See `CMakePresets.json` for the full list of supported platforms and compilers. See `.github/workflows/ci.yml` to see exactly how CI runners build it on different platforms.

## Collision complexity
The following table shows the collider types defined by this library as well as the complexity of the collisions between them in the narrow-phase. Note that Mesh and Heightfield cannot collide with themselves or each other because those cases are prohibitively complex for games (quadratic), so the library simply lets them overlap and does not support collision resolution between them. Plane vs plane is unsupported because planes can only ever be static colliders, and statics do no collide with one another.

|                 | sphere       | box          | obb          | capsule      | plane           | mesh            | heightfield     | compound               |
| --------------- | ------------ | ------------ | ------------ | ------------ | --------------- | --------------- | --------------- | ---------------------- |
| **sphere**      | O(1)         | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **box**         | O(1)&dagger; | O(1)         | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **obb**         | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **capsule**     | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **plane**       | O(1)         | O(1)         | O(1)         | O(1)         | **unsupported** | O(log T + k)    | O(c)            | &times; Q              |
| **mesh**        | O(log T + k) | O(log T + k) | O(log T + k) | O(log T + k) | O(log T + k)    | **unsupported** | **unsupported** | &times; Q              |
| **heightfield** | O(c)         | O(c)         | O(c)         | O(c)         | O(c)            | **unsupported** | **unsupported** | &times; Q              |
| **compound**    | &times; P    | &times; P    | &times; P    | &times; P    | &times; P       | &times; P       | &times; P       | **&times; P&middot;Q** |
