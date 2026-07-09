#!/usr/bin/env python

from __future__ import print_function

import os
import numpy
import pb_robot
import tampExample

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import read
from pddlstream.language.constants import print_solution
from pddlstream.language.stream import StreamInfo

EPS = 1e-5

# ---------------------------------------------------------------------------
# Push stream generators
# ---------------------------------------------------------------------------

def get_push_pose_gen():
    """Sample stable poses for a block on the given surface (mat region)."""
    def gen(block, surface):
        # Reuse the existing stable pose generator for the table surface
        stable_gen = tampExample.primitives.get_stable_gen_table([surface])
        for pose, in stable_gen(block, surface):
            yield (pose,)
    return gen


def get_push_motion_fn(robot, fixed):
    """
    Plan a straight-line push trajectory for the end effector.
    The EE approaches the side of the block and moves horizontally
    to the goal pose in a straight line, hand closed throughout.
    Returns a pddlstream-compatible function: (arm, block, p1, p2) -> (traj,) or None
    """
    def fn(arm, block, pose1, pose2):
        tf1 = pose1.pose
        tf2 = pose2.pose
        start_pos = tf1[:3, 3]
        goal_pos  = tf2[:3, 3]

        delta = goal_pos - start_pos
        dist  = numpy.linalg.norm(delta[:2])
        print(f'[push] start={start_pos}, goal={goal_pos}, dist={dist:.3f}')
        if dist < EPS:
            print('[push] dist too small, skipping')
            return None

        push_dir = delta / numpy.linalg.norm(delta)
        block_aabb = pb_robot.aabb.get_aabb(block)
        half_extent = (block_aabb.upper[0] - block_aabb.lower[0]) / 2.0
        contact_offset = -(half_extent + 0.01)
        ee_start = start_pos + push_dir * contact_offset
        ee_start[2] = start_pos[2]
        ee_goal = goal_pos + push_dir * contact_offset
        ee_goal[2] = goal_pos[2]
        print(f'[push] ee_start={ee_start}, ee_goal={ee_goal}')

        try:
            ee_rot = robot.arm.GetEETransform()[:3, :3]
            ee_start_pose = numpy.eye(4)
            ee_start_pose[:3, :3] = ee_rot
            ee_start_pose[:3, 3] = ee_start
            ee_goal_pose = numpy.eye(4)
            ee_goal_pose[:3, :3] = ee_rot
            ee_goal_pose[:3, 3] = ee_goal

            q_start_vals = robot.arm.ComputeIK(ee_start_pose)
            q_goal_vals  = robot.arm.ComputeIK(ee_goal_pose)
            print(f'[push] IK start={q_start_vals}, goal={q_goal_vals}')

            if q_start_vals is None or q_goal_vals is None:
                print('[push_motion] IK failed')
                return None

            q_start = pb_robot.vobj.BodyConf(robot.arm, q_start_vals)
            q_goal  = pb_robot.vobj.BodyConf(robot.arm, q_goal_vals)

            free_motion_fn = tampExample.primitives.get_free_motion_gen(fixed)
            print('[push] calling free_motion_fn...')
            result = free_motion_fn(robot.arm, q_start, q_goal)
            print(f'[push] free_motion result={result}')
            if result is None:
                return None

            traj, = result
            return (traj,)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return None

    return fn


# ---------------------------------------------------------------------------
# Problem setup
# ---------------------------------------------------------------------------

def pddlstream_from_problem(robot, movable, mat):
    setting_dir = os.path.dirname(os.path.realpath(__file__))

    domain_pddl = read(os.path.join(setting_dir, 'domain_testcase.pddl'))
    stream_pddl = read(os.path.join(setting_dir, 'stream_testcase.pddl'))
    constant_map = {}

    fixed = tampExample.misc.get_fixed(robot, movable)

    stream_map = {
        'sample-push-pose':  from_gen_fn(get_push_pose_gen()),
        'plan-push-motion':  from_fn(get_push_motion_fn(robot, fixed)),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map


def setup_scene(robot):
    robot_pose = numpy.eye(4)
    robot_pose[2, 3] -= 0.1
    robot.set_transform(robot_pose)

    setting_dir = os.path.dirname(os.path.realpath(__file__))
    models_path = os.path.join(setting_dir, '../../models')

    # Floor / table
    floor_path = os.path.join(models_path, 'short_floor.urdf')
    table = pb_robot.body.createBody(floor_path)
    table.set_point([0.2, 0, -0.11])

    # Mat — goal region
    mat_path = os.path.join(models_path, 'mat.urdf')
    mat = pb_robot.body.createBody(mat_path)
    mat.set_point([0.4, 0.35, pb_robot.placements.stable_z(mat, table) - 0.01])

    # One block to push
    block_path = os.path.join(models_path, 'block_a.urdf')
    block = pb_robot.body.createBody(block_path)

    table_top_z = -0.11 + 0.05 + EPS
    block.set_point([0.3, 0.0, table_top_z])

    return block, table, mat


def build_init(robot, block, table, mat):
    table_top_z = -0.11 + 0.05 + EPS

    conf = pb_robot.vobj.BodyConf(robot, robot.arm.GetJointValues())
    block_pose  = pb_robot.vobj.BodyPose(block,  block.get_transform())
    table_pose  = pb_robot.vobj.BodyPose(table,  table.get_transform())
    mat_pose    = pb_robot.vobj.BodyPose(mat,    mat.get_transform())

    init = [
        ('CanMove',),
        ('Arm', robot.arm),
        ('HandEmpty',),
        ('Block', block),
        ('Table', table),
        ('Region', mat),
        ('Pose', block, block_pose),
        ('AtPose', block, block_pose),
        ('Pose', table, table_pose),
        ('AtPose', table, table_pose),
        ('Pose', mat, mat_pose),
        ('AtPose', mat, mat_pose),
        ('On', block, table),
        ('Supported', block, block_pose, table),
    ]

    # Goal: block on the mat
    goal = ('and', ('On', block, mat))

    return init, goal


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.set_default_camera()

    robot = pb_robot.panda.Panda()
    robot.arm.hand.Close()  # hand closed for pushing

    block, table, mat = setup_scene(robot)
    domain_pddl, constant_map, stream_pddl, stream_map = pddlstream_from_problem(robot, [block], mat)
    init, goal = build_init(robot, block, table, mat)

    saved_world = pb_robot.utils.WorldSaver()

    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', list(stream_map.keys()))


    # --- Diagnostics ---
    print('\n--- Testing sample-push-pose ---')
    gen = get_push_pose_gen()(block, mat)
    for i, result in enumerate(gen):
        print(f'  sampled pose {i}:', result)
        if i >= 2: break

    print('\n--- Testing plan-push-motion ---')
    fixed_test = tampExample.misc.get_fixed(robot, [block])
    push_fn = get_push_motion_fn(robot, fixed_test)
    block_pose_test = pb_robot.vobj.BodyPose(block, block.get_transform())
    goal_tf = block.get_transform().copy()
    goal_tf[0, 3] += 0.1
    goal_tf[1, 3] += 0.35
    goal_pose_test = pb_robot.vobj.BodyPose(block, goal_tf)
    result = push_fn(robot.arm, block, block_pose_test, goal_pose_test)
    print('  push motion result:', result)
    print('--- End Diagnostics ---\n')

    stream_info = {}
    pddlstream_problem = (domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    # After setup_scene, before solve_focused
print('mat name:', mat.get_name())
print('table name:', table.get_name())

# Check what fixed surfaces get_fixed returns
fixed = tampExample.misc.get_fixed(robot, [block])
print('fixed surfaces:', [f.get_name() for f in fixed])

# Try pose gen manually
gen = get_push_pose_gen()(block, mat)
results = []
for i, r in enumerate(gen):
    results.append(r)
    if i >= 5: break
print('poses from mat:', results)

# Also try with table
gen2 = get_push_pose_gen()(block, table)
for i, r in enumerate(gen2):
    print('pose from table:', r)
    if i >= 2: break

    solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=numpy.inf)
    print_solution(solution)
    plan, cost, evaluations = solution
    print()

    if plan is None:
        print('No plan found')
    else:
        saved_world.restore()
        input('Execute?')
        tampExample.misc.ExecuteActions(plan)

    input('Finish?')
    pb_robot.utils.disconnect()