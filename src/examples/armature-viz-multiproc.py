import matplotlib.pyplot as plt
import numpy as np

from jh1.physical.robotics import Armature
from jh1.physical.robotics.kinematics import JointVector


q = [2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]
# q = [1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]
positions = Armature.forward_kinematics(JointVector.from_list(q))

fixed_mask = np.array([1, 0, 0, 1, 0, ])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '-QDo', )
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_box_aspect([1, 1, 1])
plt.show()
