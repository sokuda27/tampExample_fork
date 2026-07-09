#!/usr/bin/env python

'''
Very simple PDDLStream problem with the Franka Emika Panda
'''

from __future__ import print_function

import os
import numpy
import IPython
import pb_robot
import tampExample
import pybullet as p
import time

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn, from_test 
from pddlstream.utils import read
from pddlstream.language.constants import print_solution
from pddlstream.language.stream import StreamInfo

def pddlstream_from_problem(robot, movable):
    '''Create all of the PDDL pieces, i.e. set files, define init and goal, 
    specify the stream map'''
    setting_dir = os.path.dirname(os.path.realpath(__file__))

    domain_pddl = read(os.path.join(setting_dir, 'domain.pddl'))
    stream_pddl = read(os.path.join(setting_dir, 'stream.pddl'))
    constant_map = {}

    print('Robot:', robot)
    conf = pb_robot.vobj.BodyConf(robot, robot.arm.GetJointValues())
    init = [('CanMove',),
            ('Arm', robot.arm),
            ('Conf', conf),
            ('AtConf', robot.arm, conf),
            ('HandEmpty', robot.arm)] 

    fixed = tampExample.misc.get_fixed(robot, movable)
    print('Movable:', [m.get_name() for m in movable])
    print('Fixed:', [f.get_name() for f in fixed])
    for body in movable:
        pose = pb_robot.vobj.BodyPose(body, body.get_transform())
        init += [('Pose', body, pose),
                 ('AtPose', body, pose), 
                 ('Block', body)]

        for surface in fixed:
            if 'flat' in surface.get_name() and 'block' in body.get_name():
                init += [('Stackable', body, surface), 
                         ('Supported', body, pose, surface)]
            if 'mat' in surface.get_name() and 'block' in body.get_name():
                init += [('Stackable', body, surface)]

    init += [('Region', fixed[0]), ('Region', fixed[1])]
    goal = ('and', ('On', movable[0], fixed[1])) # Places object on red mat

    stream_map = {
        'sample-pose':        from_gen_fn(tampExample.primitives.get_push_pose_gen(fixed)),
        'sample-push':        from_fn(tampExample.primitives.get_push_fn(fixed)),
        'plan-free-motion':   from_fn(tampExample.primitives.get_free_motion_gen(fixed)),
        'test-pose-cfree':    from_test(tampExample.primitives.pose_collision_test),
        'test-traj-cfree':    from_test(tampExample.primitives.traj_collision_test),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def setup_pb_scene(robot):
    '''Add everything in pybullet'''
    robot_pose = numpy.eye(4)
    robot_pose[2, 3] -= 0.1
    robot.set_transform(robot_pose)
    
    # Define where models are
    setting_dir = os.path.dirname(os.path.realpath(__file__))
    models_path = os.path.join(setting_dir, '../../models')

    # Create floor    
    floor_path = os.path.join(models_path, 'short_floor.urdf')
    table = pb_robot.body.createBody(floor_path)
    table.set_point([0.2, 0, -0.11])

    # Create mat
    mat_path = os.path.join(models_path, 'mat.urdf')
    mat = pb_robot.body.createBody(mat_path)
    mat.set_point([0.5, 0.35, pb_robot.placements.stable_z(mat, table)-0.01])

    # Create and place blocks
    # block_names = ['block_a.urdf', 'block_b.urdf', 'block_c.urdf']
    block_names = ['block_oblong.urdf']
    blocks = []
    for bname in block_names:
        block_path = os.path.join(models_path, bname)
        block = pb_robot.body.createBody(block_path)
        blocks.append(block)

    table_top_z = -0.06
    # Set the initial positions as a partial tower.
    blocks[0].set_point([0.6, 0.0, table_top_z + 0.02])

    return blocks

#######################################################

if __name__ == '__main__':
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.set_default_camera()

    robot = pb_robot.panda.Panda()
    robot.arm.hand.Open()
    blocks = setup_pb_scene(robot)

    saved_world = pb_robot.utils.WorldSaver()
    pddlstream_problem = pddlstream_from_problem(robot, blocks)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    print('Synthesizers:', stream_map.keys()) 

    stream_info = {'test-pose-cfree': StreamInfo(negate=True),
                   'test-traj-cfree': StreamInfo(negate=True)}
    
    IPython.embed()

    solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=numpy.inf)
    print_solution(solution)
    plan, cost, evaluations = solution
    print('\n')

    if plan is None:
        print("No plan found")
    else:
        saved_world.restore()
        input("Execute?")
        tampExample.misc.ExecuteActions(plan, robot=robot)
        # tampExample.misc.ExecuteActions(plan)

    input('Finish?')
    pb_robot.utils.disconnect()
