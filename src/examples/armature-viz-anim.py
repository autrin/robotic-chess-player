from jh1.physical import Orchestrator
from jh1.physical.topology import SQUARE_IK_LOOKUP
from jh1.robotics import Armature
from jh1.utils.visualize import animate_joint_vectors

if __name__ == '__main__':
    armature = Armature(None)
    orchestrator = Orchestrator(armature)
    orchestrator.pick_and_drop_sequence(
        start_square="a1",
        end_square="c6"
    )

    # joint_vectors = [
    #     SQUARE_IK_LOOKUP['home'].angles_ik,
    #     SQUARE_IK_LOOKUP['a1'].angles_ik,
    #     SQUARE_IK_LOOKUP['h1'].angles_ik,
    #     SQUARE_IK_LOOKUP['h8'].angles_ik,
    #     SQUARE_IK_LOOKUP['a8'].angles_ik,
    #     SQUARE_IK_LOOKUP['a1'].angles_ik,
    #     SQUARE_IK_LOOKUP['c1_up'].angles_ik,
    #     SQUARE_IK_LOOKUP['c1'].angles_ik,
    #     SQUARE_IK_LOOKUP['c1_up'].angles_ik,
    #     SQUARE_IK_LOOKUP['f4_up'].angles_ik,
    #     SQUARE_IK_LOOKUP['f4'].angles_ik,
    #     SQUARE_IK_LOOKUP['f4_up'].angles_ik,
    #     SQUARE_IK_LOOKUP['home'].angles_ik,
    #
    #     # JointVector.from_topic([2.113312069569723, -1.2614153188518067, 0.6471139192581177, -2.404459138909811, -1.5351746718036097, 1.085855484008789]),
    #     # JointVector.from_topic([1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003, -1.6312678495990198, 1.6715844869613647]),
    #     # JointVector.from_topic([1.0309518019305628, -0.5775613945773621, 1.2293174266815186, -2.0083195171751917, -1.6719935576068323, 1.6715705394744873]),
    #     # JointVector.from_topic([1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631, -1.5819533506976526, 1.3885889053344727]),
    #     # JointVector.from_topic([2.4494758288012903, -1.842133184472555, 1.006606101989746, -2.13765873531484,-1.580822769795553, 1.3886008262634277])
    #
    #     # JointVector.from_list([2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]),
    #     # # JointVector.from_list([1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]),
    #     # # JointVector.from_list([1.7860, -1.0432, 1.3300, -2.3513, -1.5691, 2.0217])
    #     # Armature.inverse_kinematics(np.array([0, -0.7, 0.4])),
    #     # Armature.inverse_kinematics(np.array([0, -0.2, 0.4])),
    #     # Armature.inverse_kinematics(np.array([0.5, -0.2, 0.4])),
    #     # Armature.inverse_kinematics(np.array([0.5, -0.7, 0.4]))
    # ]
    #
    # animate_joint_vectors(joint_vectors)
