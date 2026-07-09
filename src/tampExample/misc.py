#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Various utility functions
'''

import numpy
import pb_robot
import pybullet as p
import time
import tampExample

def get_fixed(robot, movable):
    '''Given the robot and movable objects, return all other 
    objects in the scene, which are then by definition, the fixed objects'''
    rigid = [body for body in pb_robot.utils.get_bodies() if body.id != robot.id]
    movable_ids = [m.id for m in movable]
    fixed = [body for body in rigid if body.id not in movable_ids]
    return fixed

# def execute_position_path_dynamics(arm, path, steps_per_waypoint=30, force=87, timestep=1./240):
#     joint_indices = arm.jointsID   # <-- corrected attribute name
#     for q_target in path:
#         p.setJointMotorControlArray(
#             arm.id, joint_indices, p.POSITION_CONTROL,
#             targetPositions=list(q_target),
#             forces=[force] * len(joint_indices)
#         )
#         for _ in range(steps_per_waypoint):
#             p.stepSimulation()
#             time.sleep(timestep)

# def ExecuteActions(plan):
#     for name, args in plan:
#         pb_robot.viz.remove_all_debug()
#         bodyNames = [args[i].get_name() for i in range(len(args)) if isinstance(args[i], pb_robot.body.Body)]
#         txt = '{} - {}'.format(name, bodyNames)
#         pb_robot.viz.add_text(txt, position=(0, 0.25, 0.5), size=2)

#         executionItems = args[-1]
#         for e in executionItems:
#             if name == 'push':   # match whatever your domain.pddl actually calls this action
#                 execute_position_path_dynamics(e.manip, e.path)
#             else:
#                 e.simulate()
#             input("Next?")

# # ORIGINAL
# def ExecuteActions(plan): 
#     '''Iterate through the plan, simulating each action'''
#     for name, args in plan:
#         pb_robot.viz.remove_all_debug()
#         bodyNames = [args[i].get_name() for i in range(len(args)) if isinstance(args[i], pb_robot.body.Body)]
#         txt = '{} - {}'.format(name, bodyNames)
#         pb_robot.viz.add_text(txt, position=(0, 0.25, 0.5), size=2)

#         executionItems = args[-1]
#         for e in executionItems:
#             e.simulate()
#             input("Next?")

# # IMPEDANCE
def ExecuteActions(plan, robot=None):
    for name, args in plan:
        pb_robot.viz.remove_all_debug()
        bodyNames = [args[i].get_name() for i in range(len(args)) if isinstance(args[i], pb_robot.body.Body)]
        txt = '{} - {}'.format(name, bodyNames)
        pb_robot.viz.add_text(txt, position=(0, 0.25, 0.5), size=2)

        executionItems = args[-1]
        for e in executionItems:
            if name == 'push' and robot is not None:
                tampExample.primitives.run_impedance_push(robot, e.path)
            else:
                e.simulate()
            input("Next?")

def ComputePrePose(og_pose, directionVector, relation=None):
    '''Given a pose, compute the "backed-up" pose, i.e. the pose offset
    by the desired direction vector
    @param og_pose Transform to offset from
    @param directionVector 3D vector to offset by
    @param relation Optional parameter corresponding to a grasp
    @return prepose Transform of og_pose offset by directionVector'''
    backup = numpy.eye(4)
    backup[0:3, 3] = directionVector
    prepose = numpy.dot(og_pose, backup)
    if relation is not None:
        prepose = numpy.dot(prepose, relation)
    return prepose
