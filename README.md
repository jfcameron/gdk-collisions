# gdk-collisions
3D collision detection and resolution library for games: spheres, boxes, capsules, planes, meshes, heightfields and compounds of them, moving through a scene that keeps them apart, reports what touched, and answers rays and volume queries.

API documentation: https://jfcameron.github.io/gdk-collisions/

## Getting Started
Install cmake 3.21 or greater, and a c++20 compatible compiler
```
# initialize the tree
git submodule update --init --recursive

# generate the script to build the library and its tests on linux using gcc
cmake --preset linux-gcc

# build the library and tests
cmake --build --preset linux-gcc

# run the test suite
ctest --preset linux-gcc
```
See `CMakePresets.json` for the full list of supported platforms and compilers. See `.github/workflows/ci.yml` to see exactly how CI runners build it on different platforms.

To use it from another project, add it with `add_subdirectory` and link `gdk::collisions`, or install it and `find_package(gdk-collisions)`.

## Usage Example
```cpp
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>

using namespace gdk::collisions;

const auto pScene = make_scene(
    [](const collision_event &aEvent) { /* enter, stay or exit between two solid colliders */ },
    [](const trigger_event &aEvent) { /* the same for trigger volumes */ });

const auto pGround = pScene->make_static_plane_collider(matrix4x4_type::identity);

const auto pBall = pScene->make_sphere_collider();
pBall->set_position({0, 2, 0});

while (running) {
    pBall->add_velocity({0, -9.8f, 0}); // what the ball wants to travel this update, per second

    pScene->update(1.0f / 60);
    pScene->process_events();
}
```

## Overview
### Colliders
A scene makes every collider and hands back a `shared_ptr`. The scene only watches its colliders: dropping the last pointer removes one, from any thread.

There are three kinds of body:
- **dynamic** colliders are moved by the scene and pushed out of whatever they meet;
- **static** colliders (`make_static_*`) are placed once, by a rigid transform, and never move. Scale the geometry, not the transform: a transform that scales, shears or reflects throws;
- **kinematic** colliders (`set_kinematic`) travel their whole velocity every update. They push dynamic colliders aside, nothing pushes them, and they never meet one another, so several can make up one moving body.

Each kind can instead be a **trigger** (`make_*_trigger`): a volume that reports overlaps and blocks nothing.

### Velocity
Velocity is what a collider wants to travel during the next `update`, in units per second, and the update consumes it: afterwards `velocity()` is zero, and `resolved_velocity()` is what it became after every contact. Add it again each frame. Angular velocity works the same way.

What a contact does to velocity is the collider's response handler's choice. `response_handlers::slide_preserving_speed` (the default) slides along the surface at the original speed, `slide_projecting` keeps only the speed the slide leaves, and `null_opt` changes nothing. A handler of your own returns a change to the velocity, not a new velocity.

How far a collider is pushed out of an overlap is set by `set_inverse_overlap_weight`: two colliders share a separation in proportion to their weights, and a weight of 0 never moves.

### Layers
`set_collision_layers(on, collides_with)` puts a collider on a set of layers. Two colliders meet only when each collides with a layer the other is on.

### Events and contacts
`update` records what touched; `process_events` hands it to the observers as enter, stay and exit events. `contacts()` and `contacts_for(collider)` give the solid contacts of the most recent update with their normal, point and depth.

### Queries
- `raycast` finds the nearest collider along a ray, skipping one collider if asked.
- `raycast_static` finds the nearest static collider.
- `overlaps_static` asks whether any static collider reaches into an upright cylinder, such as the room a character needs to stand, or into a parallelepiped, such as the space between two points.

### Kernels
A scene moves its colliders and decides what their contacts mean. Its spatial work is done by its kernels:
- which bodies could meet;
- whether they do;
- what rays and volumes reach.

The kernels are given to `make_scene`:
- `make_std_kernels` gives uniform grids, bounding volume hierarchies and the library's own narrow phase, on the standard library alone. It is the default.
- `make_null_kernels` finds nothing: bodies pass through each other and rays strike nothing.

Kernels are an interface, `scene_kernels`, that anyone can implement without changing this library. One might put the work on a GPU, for instance.
- **What they read:** the scene hands its kernels flat tables of bodies and their parts (`gdk/collisions/kernels/tables.h`). Every kernel reads the same data.
- **Partial kernels:** a kernel that does only some of the work holds a `make_std_kernels()` and passes the rest to it.
- **Testing a kernel:** the conformance suite, the `gdkcollisions_kernel_conformance` target, holds a kernel to the interface's contract. Link it into your kernel's tests and define the three functions in `test/conformance/conformance.h`.

### Tuning and threads
- **Scene tuning:** `scene_policy` holds steps per update, the advancement iteration limit, and the tolerances every kernel follows.
- **Std kernel tuning:** `std_kernels_policy` holds the grid's cell size, the parallel chunking, and the narrow phase's own epsilons.
- **The defaults** suit bodies about a unit across. Both factories throw on a policy they cannot run with.

Threads:
- **Updates:** a scene is updated, and its colliders are made and changed, from one thread at a time.
- **Queries:** `raycast` and `overlaps_static` may run on several threads at once between updates.
- **Lending threads:** give `make_std_kernels` a `task_dispatcher_type` (see jfc-dispatch), and large updates hand their binning, gathering and pair tests to your threads. Without one, every update runs serially.

For results that are the same on every machine, build with `GDK_MATH_STRICT_FP`, which gdk-math applies to everything built on it, this library included.

## Collision complexity
The following table shows the collider types defined by this library as well as the complexity of the collisions between them in the std kernels' narrow phase.

Mesh and heightfield cannot collide with themselves or each other: those cases are prohibitively complex for games (quadratic), so the library lets them overlap and does not resolve them. Plane vs plane is unsupported because planes can only be static, and statics do not collide with one another.

|                 | sphere       | box          | obb          | capsule      | plane           | mesh            | heightfield     | compound               |
| --------------- | ------------ | ------------ | ------------ | ------------ | --------------- | --------------- | --------------- | ---------------------- |
| **sphere**      | O(1)         | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **box**         | O(1)&dagger; | O(1)         | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **obb**         | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **capsule**     | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)&dagger; | O(1)            | O(log T + k)    | O(c)            | &times; Q              |
| **plane**       | O(1)         | O(1)         | O(1)         | O(1)         | **unsupported** | O(log T + k)    | O(T)            | &times; Q              |
| **mesh**        | O(log T + k) | O(log T + k) | O(log T + k) | O(log T + k) | O(log T + k)    | **unsupported** | **unsupported** | &times; Q              |
| **heightfield** | O(c)         | O(c)         | O(c)         | O(c)         | O(T)            | **unsupported** | **unsupported** | &times; Q              |
| **compound**    | &times; P    | &times; P    | &times; P    | &times; P    | &times; P       | &times; P       | &times; P       | **&times; P&middot;Q** |

- **T** is a mesh's or heightfield's triangle count.
- **k** is the number of triangles near the other collider.
- **c** is the number of heightfield cells beneath the other collider.
- **P** and **Q** are the part counts of the compounds involved.
- **&dagger;** marks pairs found by conservative advancement: a few distance queries per step, bounded by `std_kernels_policy::SWEEP_MAX_STEPS`.
