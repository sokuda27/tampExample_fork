#!/usr/bin/env python

from __future__ import print_function

import os
import shutil
import numpy
import pb_robot
import tampExample

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn, from_test 
from pddlstream.utils import read
from pddlstream.language.constants import print_solution
from pddlstream.language.stream import StreamInfo

EPS = 1e-5

def pddlstream_from_problem(robot, movable):
    setting_dir = os.path.dirname(os.path.realpath(__file__))

    domain_pddl = read(os.path.join(setting_dir, 'domain_stacking.pddl'))
    stream_pddl = read(os.path.join(setting_dir, 'stream_stacking.pddl'))
    constant_map = {}
    
    fixed = tampExample.misc.get_fixed(robot, movable)
    stream_map = {
        'sample-pose-table': from_gen_fn(tampExample.primitives.get_stable_gen_table(fixed)),
        'sample-pose-block': from_fn(tampExample.primitives.get_stable_gen_block()),
        'sample-grasp': from_gen_fn(tampExample.primitives.get_grasp_gen()),
        'inverse-kinematics': from_fn(tampExample.primitives.get_ik_fn(fixed)), 
        'plan-free-motion': from_fn(tampExample.primitives.get_free_motion_gen(fixed)),
        'plan-holding-motion': from_fn(tampExample.primitives.get_holding_motion_gen(fixed)),
        'test-traj-cfree': from_test(tampExample.primitives.traj_collision_test),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map

def setup_blocks_and_robot(robot):
    # Adjust robot position such that measurements match real robot reference frame
    robot_pose = numpy.eye(4)
    robot_pose[2, 3] -= 0.1
    robot.set_transform(robot_pose)

    block_names = ['block_a.urdf', 'block_b.urdf', 'block_c.urdf']
    block_heights = [0.04, 0.08, 0.12]
    blocks = []

    # copying files to where pb_robot expects them to be
    setting_dir = os.path.dirname(os.path.realpath(__file__))
    models_path = os.path.join(setting_dir, '../../models')
    
    for bname in block_names:
        block_path = os.path.join(models_path, bname)
        block = pb_robot.body.createBody(block_path)
        blocks.append(block)
    
    floor_path = os.path.join(models_path, 'short_floor.urdf')
    table = pb_robot.body.createBody(floor_path) 
    table.set_point([0.2, 0, -0.11])
    
    return blocks

def initialize_in_towers(robot, blocks):
    table_top_z = -0.11 + 0.05 + EPS 

    # Set the initial positions as a partial tower.
    blocks[0].set_point([0.6, 0.0, table_top_z + 0.02])
    blocks[1].set_point([0.7, 0.0, table_top_z + 0.04])
    blocks[2].set_point([0.7, 0.0, table_top_z + 0.14])

    print('Robot:', robot)
    conf = pb_robot.vobj.BodyConf(robot, robot.arm.GetJointValues())
    init = [('CanMove',),
            ('Arm', robot.arm),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]

    fixed = tampExample.misc.get_fixed(robot, blocks)
    print('Blocks:', [b.get_name() for b in blocks])
    print('Fixed:', [f.get_name() for f in fixed])
    for body in blocks:
        pose = pb_robot.vobj.BodyPose(body, body.get_transform())
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose),
                 ('Block', body)]

    init += [('On', blocks[0], fixed[0]), 
             ('On', blocks[2], blocks[1]), 
             ('On', blocks[1], fixed[0])]

    for surface in fixed:
        if 'flat' in surface.get_name():
            pose = pb_robot.vobj.BodyPose(surface, surface.get_transform())
            init += [('Table', surface), ('Pose', surface, pose), ('AtPose', surface, pose)]
    
    # Tower with three blocks.
    goal = ('and', ('On', blocks[0], fixed[0]), 
                   ('On', blocks[1], blocks[0]), 
                   ('On', blocks[2], blocks[1]))  
    return init, goal

def initialize_flat(robot, blocks):
    table_top_z = -0.11 + 0.05 + EPS

    # Set the initial positions as a partial tower.
    blocks[0].set_point([0.6, 0.0, table_top_z + 0.02])
    blocks[1].set_point([0.7, 0.0, table_top_z + 0.04])
    blocks[2].set_point([0.8, 0.0, table_top_z + 0.06])

    print('Robot:', robot)
    conf = pb_robot.vobj.BodyConf(robot, robot.arm.GetJointValues())
    init = [('CanMove',),
            ('Arm', robot.arm),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]

    fixed = tampExample.misc.get_fixed(robot, blocks)
    print('Blocks:', [b.get_name() for b in blocks])
    print('Fixed:', [f.get_name() for f in fixed])
    for body in blocks:
        pose = pb_robot.vobj.BodyPose(body, body.get_transform())
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose),
                 ('Block', body)]

    init += [('On', blocks[0], fixed[0]), 
             ('On', blocks[2], fixed[0]), 
             ('On', blocks[1], fixed[0])]

    for surface in fixed:
        if 'flat' in surface.get_name():
            pose = pb_robot.vobj.BodyPose(surface, surface.get_transform())
            init += [('Table', surface), ('Pose', surface, pose), ('AtPose', surface, pose)]
    
    # Tower with three blocks.
    goal = ('and', ('On', blocks[0], fixed[0]), 
                   ('On', blocks[1], blocks[0]), 
                   ('On', blocks[2], blocks[1]))  
    return init, goal

#######################################################

if __name__ == '__main__':
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.set_default_camera()

    robot = pb_robot.panda.Panda()
    robot.arm.hand.Open()
    blocks = setup_blocks_and_robot(robot)

    domain_pddl, constant_map, stream_pddl, stream_map = pddlstream_from_problem(robot, blocks)
    init, goal = initialize_in_towers(robot, blocks)
    # init, goal = initialize_flat(robot, blocks) 
    saved_world = pb_robot.utils.WorldSaver()

    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    print('Synthesizers:', stream_map.keys()) 
    
    stream_info = {'test-traj-cfree': StreamInfo(negate=True)}
    pddlstream_problem = (domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=numpy.inf)
    print_solution(solution)
    plan, cost, evaluations = solution
    print('\n')

    if plan is None:
        print("No plan found")
    else:
        saved_world.restore()
        input("Execute?")
        tampExample.misc.ExecuteActions(plan)

    input('Finish?')
    pb_robot.utils.disconnect()
