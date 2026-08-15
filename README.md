# gdk-collisions

3D collision detection and resolution in C++17

## Collider pairs

Cost of narrow-phase tests between collider types. Note that Mesh and Heightfield cannot collide with themselves or eachother. This is because those cases are prohibitively complex for games (quadratic), so the library simply lets them overlap and does not support collision resolution between them. Plane vs plane is unsupported because planes can only ever be static colliders, and statics do no collide with one another.

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
