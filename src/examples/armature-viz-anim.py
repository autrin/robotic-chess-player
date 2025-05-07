from jh1.core import Orchestrator
from jh1.robotics import Skeleton
from jh1.topology import WAYPOINT_TABLE
from jh1.utils.visualize import animate_joint_vectors

if __name__ == '__main__':
    # armature = Skeleton(None)
    # orchestrator = Orchestrator(armature)

    joint_vectors = [
        WAYPOINT_TABLE['home'].jv,
        WAYPOINT_TABLE['discard_up'].jv,
        WAYPOINT_TABLE['discard'].jv,
        # WAYPOINT_TABLE['a1'].jv,
        # WAYPOINT_TABLE['h1'].jv,
        # WAYPOINT_TABLE['h8'].jv,
        # WAYPOINT_TABLE['a8'].jv,
        WAYPOINT_TABLE['a1'].jv,
        # WAYPOINT_TABLE['c1_up'].jv,
        WAYPOINT_TABLE['c1'].jv,
        # WAYPOINT_TABLE['c1_up'].jv,
        # WAYPOINT_TABLE['f4_up'].jv,
        # WAYPOINT_TABLE['f4'].jv,
        # WAYPOINT_TABLE['f4_up'].jv,
        # WAYPOINT_TABLE['home'].jv,

        # JointVector.from_topic([2.113312069569723, -1.2614153188518067, 0.6471139192581177, -2.404459138909811, -1.5351746718036097, 1.085855484008789]),
        # JointVector.from_topic([1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003, -1.6312678495990198, 1.6715844869613647]),
        # JointVector.from_topic([1.0309518019305628, -0.5775613945773621, 12293174266815186, -2.0083195171751917, -1.6719935576068323, 1.6715705394744873]),
        # JointVector.from_topic([1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631, -1.5819533506976526, 1.3885889053344727]),
        # JointVector.from_topic([2.4494758288012903, -1.842133184472555, 1.006606101989746, -2.13765873531484,-1.580822769795553, 1.3886008262634277])

        # JointVector.from_list([2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]),
        # # JointVector.from_list([1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]),
        # # JointVector.from_list([1.7860, -1.0432, 1.3300, -2.3513, -1.5691, 2.0217])
        # Armature.inverse_kinematics(np.array([0, -0.7, 0.4])),
        # Armature.inverse_kinematics(np.array([0, -0.2, 0.4])),
        # Armature.inverse_kinematics(np.array([0.5, -0.2, 0.4])),
        # Armature.inverse_kinematics(np.array([0.5, -0.7, 0.4]))
    ]

    animate_joint_vectors(joint_vectors)
