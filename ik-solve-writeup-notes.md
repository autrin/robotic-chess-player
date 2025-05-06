I've defined various matrix transformations in affine.py. These work in both 2D homogeneous (Mat3x3)
and 3D homogeneous (Mat4x4). There are also functions to generate SO(3) matrices for linear rotation
in 3D.

_armature_def contains measurements (size, linkage, local rotational axis) of the UR10e robot. These
can be found from resources/ur10e_mjcf/ur10e.xml. The quat of each linkage determines the rotation.

Remember that quaternions are defined as cos(theta/2) + sin(theta/2) \* (xi, yj, zk), representing a
rotation of theta CCW around the axis denoted by the vector [x, y, z]. For example, (1, 1, 0, 0)
normalized is 1/sqrt2 + 1/sqrt2\*i + 0\*j + 0\*k, which represents a 90 degree roll.

Forward kinematics is implemented in \_forward\_kinematics.py, which returns a position array given
a 6-dimensional angle configuration vector.

Now, we need semantically associate the FK method with the UR10e Armature class. I define a joint
vector dataclass that forces named parameterization of the underlying 6-dim vector. This is separate
from the end-effector gripper span. Then, define a pair of FK and IK static methods in Armature:

```
forward_kinematics(q: JointVector) -> NDArray[Vec3]
inverse_kinematics(target: Vec3) -> JointVector
```