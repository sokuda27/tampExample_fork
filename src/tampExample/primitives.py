#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Samplers needed for PDDLStream. Implements grasping, placing, planning, cutting and collision-checking
'''

import numpy
import pybullet as p
from . import pb_robot
import tampExample
import math

DEBUG_FAILURE = False 

# DEBUGGING FUNCTIONS
def draw_pose_debug(pos, quat, axis_len=0.08, label=None):
    '''Draw an RGB frame (x=red, y=green, z=blue) at pos/quat in the pybullet GUI'''
    rot = numpy.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
    colors = [(1,0,0), (0,1,0), (0,0,1)]
    for i, color in enumerate(colors):
        axis_vec = rot[:, i] * axis_len
        p.addUserDebugLine(pos, pos + axis_vec, lineColorRGB=color, lineWidth=3, lifeTime=0)
    if label:
        p.addUserDebugText(label, pos, textColorRGB=(1,1,1), textSize=1.2, lifeTime=0)


# OLD GRASPING
def get_grasp_gen():
    '''Generate function for sampling grasps'''
    # I opt to use TSR to define grasp sets but you could replace this
    # with your favorite grasp generator
    def gen(arm, body):
        '''Sample collision-free reachable grasps for body'''
        for _ in range(10):
            grasp_tsr = pb_robot.panda_grasp_sets.box_grasp(body)
            # Only use a top grasp.
            top_grasp_ix = 2
            sampled_tsr = grasp_tsr[top_grasp_ix]
            grasp_worldF = sampled_tsr.sample()
            grasp_objF = numpy.dot(numpy.linalg.inv(body.get_transform()), grasp_worldF)
            body_grasp = pb_robot.vobj.BodyGrasp(body, grasp_objF, arm)
            yield (body_grasp,)
    return gen

def get_height(body):
    '''Compute the height of a body'''
    collision_data = p.getCollisionShapeData(body.id, body.base_link)[0]
    dimensions = collision_data[3]
    return dimensions[2]

def get_stable_gen_table(fixed=None):
    '''Generate function for sampling poses'''
    def gen(body, surface, surface_pose=None):
        '''Sample poses for the body that are on top of the surface'''
        for _ in range(10):
            pose = pb_robot.placements.sample_placement(body, surface) 
            if (pose is None) or any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
                continue
            body_pose = pb_robot.vobj.BodyPose(body, pb_robot.geometry.tform_from_pose(pose))
            yield (body_pose,)
    return gen

def get_stable_gen_block():
    '''Generate function for sampling poses on top of blocks'''
    def fn(body, surface, surface_pose):
        '''Sample poses for one block (body) to be on another block (surface)'''
        pos, orn = pb_robot.geometry.pose_from_tform(surface_pose.pose)

        height_body = get_height(body)
        height_surface = get_height(surface)
        z_disp = (height_body+height_surface)/2

        pose = ((pos[0], pos[1], pos[2] + z_disp), orn) 
        body_pose = pb_robot.vobj.BodyPose(body, pb_robot.geometry.tform_from_pose(pose))
        return (body_pose,)
    return fn

def get_ik_fn(fixed=None, num_attempts=10):
    '''Generate function for solving ik for grasping and placing'''
    def fn(arm, body, pose, grasp):
        '''Plan the configuration and path needed to grasp/ungrasp a body at pose with grasp'''
        obstacles = fixed + [body]
        obj_worldF = pose.pose
        grasp_worldF = numpy.dot(obj_worldF, grasp.grasp_objF)
        approach_tform = tampExample.misc.ComputePrePose(grasp_worldF, [0, 0, -0.05])

        for _ in range(num_attempts):
            q_approach = arm.ComputeIK(approach_tform)
            if (q_approach is None) or not arm.IsCollisionFree(q_approach, obstacles=obstacles): 
                continue
            conf = pb_robot.vobj.BodyConf(arm, q_approach)
            q_grasp = arm.ComputeIK(grasp_worldF, seed_q=q_approach)
            if (q_grasp is None) or not arm.IsCollisionFree(q_grasp, obstacles=obstacles):
                continue

            #path = arm.snap.PlanToConfiguration(arm, q_approach, q_grasp, obstacles=obstacles)
            path = [q_approach, q_grasp] # Mike's edit
            if path is None:
                if DEBUG_FAILURE: input('Approach motion failed')
                continue

            command = [pb_robot.vobj.JointSpacePath(arm, path), grasp, 
                       pb_robot.vobj.JointSpacePath(arm, path[::-1])]
            return (conf, command)
        return None
    return fn

def get_free_motion_gen(fixed=None):
    '''Generate function for planning free paths, accounting for fixed obstacles'''
    def fn(arm, conf1, conf2):
        '''Plan a collision-free path from conf1 to conf2'''
        path = arm.birrt.PlanToConfiguration(arm, conf1.configuration, conf2.configuration, obstacles=fixed)
        if path is None:
            if DEBUG_FAILURE: input('Free motion failed')
            return None
        command = [pb_robot.vobj.JointSpacePath(arm, path)]
        return (command,)
    return fn

def get_holding_motion_gen(fixed=None):
    '''Generate function for planning holding paths, accounting for fixed obstacles'''
    def fn(arm, conf1, conf2, body, grasp):
        '''Plan a collision-free path from conf1 to conf2, while holding body with grasp'''
        arm.Grab(grasp.body, grasp.grasp_objF)
        path = arm.birrt.PlanToConfiguration(arm, conf1.configuration, conf2.configuration, obstacles=fixed)
        arm.Release(grasp.body)
        if path is None:
            if DEBUG_FAILURE: input('Holding motion failed')
            return None
        command = [pb_robot.vobj.JointSpacePath(arm, path)]
        return (command,)
    return fn


# NEW PUSHING FUNCTIONS
def get_push_pose_gen(fixed=None):
    def fn(body, surface):
        original_tform = body.get_transform()
        current_pose = pb_robot.vobj.BodyPose(body, body.get_transform())
        start_pos, start_orn = pb_robot.geometry.pose_from_tform(current_pose.pose)
        start_pos = numpy.array(start_pos)

        surface_aabb = pb_robot.aabb.get_aabb(surface)
        block_aabb = pb_robot.aabb.get_aabb(body)
        half_x = (block_aabb.upper[0] - block_aabb.lower[0]) / 2
        half_y = (block_aabb.upper[1] - block_aabb.lower[1]) / 2

        x_min = surface_aabb.lower[0] + half_x
        x_max = surface_aabb.upper[0] - half_x
        y_min = surface_aabb.lower[1] + half_y
        y_max = surface_aabb.upper[1] - half_y

        EPSILON = 0.002

        for _ in range(20):
            new_x = numpy.random.uniform(x_min, x_max)
            new_y = numpy.random.uniform(y_min, y_max)
            z = pb_robot.placements.stable_z(body, surface) + EPSILON

            end_pos = numpy.array([new_x, new_y, z])

            push_vec = end_pos - start_pos
            push_dist = numpy.linalg.norm(push_vec)
            if push_dist < 0.05:
                continue

            new_pose = (tuple(end_pos), tuple(start_orn))
            tform = pb_robot.geometry.tform_from_pose(new_pose)

            body.set_transform(tform)

            if any(pb_robot.collisions.pairwise_collision(body, b) for b in (fixed or [])):
                continue

            body.set_transform(original_tform)

            body_pose = pb_robot.vobj.BodyPose(body, tform)
            yield (body_pose,)
        return
    return fn

# CART IMPEDANCE
# def run_impedance_push(robot, path, 
#                         xy_stiffness=1000, z_stiffness=150, ori_stiffness=50,
#                         z_penetration=0.004):

#     controller = pb_robot.panda_controls.PandaControls(robot.arm)

#     stiffness_params = numpy.array([
#         xy_stiffness, xy_stiffness, z_stiffness,
#         ori_stiffness, ori_stiffness, ori_stiffness
#     ])

#     pose_path = []
#     for q_target in path:
#         tform = robot.arm.ComputeFK(q_target)
#         tform = numpy.array(tform, copy=True)
#         tform[2, 3] -= z_penetration
#         pose_path.append(tform)

#     controller.cartImpedancePath(pose_path, stiffness_params)

# # JOINT IMPEDANCE
def run_impedance_push(robot, path, joint_stiffness=None):
    '''
    Execute a push path via pb_robot's PandaControls.jointImpedance.
    Note: stiffness here is per-joint, not per task-space direction, so
    this does not give direct control over "soft in z / stiff in xy" the
    way cartImpedance does — compliance in world-z is only indirect.
    '''
    controller = pb_robot.panda_controls.PandaControls(robot.arm)

    n = len(path[0])
    if joint_stiffness is None:
        joint_stiffness = numpy.array([200]*n)  # tune empirically; author's comment suggests this may need to be large

    controller.jointImpedancePath(path, joint_stiffness)

def get_push_fn(fixed, target_surface=None):

    def fn(arm, body, pose1, pose2):
        p.removeAllUserDebugItems()
        start_pos, _ = pb_robot.geometry.pose_from_tform(pose1.pose)
        end_pos, _   = pb_robot.geometry.pose_from_tform(pose2.pose)
        start_pos = numpy.array(start_pos)
        end_pos   = numpy.array(end_pos)

        push_vec  = end_pos - start_pos
        push_dist = numpy.linalg.norm(push_vec)
        if push_dist < 1e-6:
            return None
        push_dir = push_vec / push_dist

        world_up  = numpy.array([0, 0, -1])
        gripper_x = push_dir.copy()
        gripper_x[2] = 0
        norm = numpy.linalg.norm(gripper_x)
        if norm < 1e-6:
            return None
        gripper_x /= norm
        gripper_y  = numpy.cross(world_up, gripper_x)
        gripper_y /= numpy.linalg.norm(gripper_y)
        gripper_z  = world_up

        rot = numpy.eye(3)
        rot[:, 0] = gripper_x
        rot[:, 1] = gripper_y
        rot[:, 2] = gripper_z

        TILT_ANGLE = math.radians(15)

        tilt_rot = numpy.array([
            [math.cos(TILT_ANGLE), 0, math.sin(TILT_ANGLE)],
            [0, 1, 0],
            [-math.sin(TILT_ANGLE), 0,  math.cos(TILT_ANGLE)],
        ])
        rot = rot @ tilt_rot
        push_orn = pb_robot.geometry.quat_from_matrix(rot)

        contact_offset = 0.1
        approach_offset = 0.08

        CLEARANCE = 0.05
        PUSH_Z = pb_robot.placements.stable_z(body, fixed[0]) + 0.1

        ee_start_pos = numpy.array([
            start_pos[0] - push_dir[0] * contact_offset,
            start_pos[1] - push_dir[1] * contact_offset,
            PUSH_Z
        ])

        ee_end_pos = numpy.array([
            end_pos[0] - push_dir[0] * contact_offset,
            end_pos[1] - push_dir[1] * contact_offset,
            PUSH_Z
        ])

        approach_pos = numpy.array([
            ee_start_pos[0] - push_dir[0] * approach_offset,
            ee_start_pos[1] - push_dir[1] * approach_offset,
            PUSH_Z
        ])

        conf1 = arm.GetJointValues()

        print("start_pos:", start_pos, "end_pos:", end_pos, "push_dir:", push_dir)
        print("approach_pos:", approach_pos, "PUSH_Z:", PUSH_Z)
        draw_pose_debug(approach_pos, push_orn, label="approach")
        draw_pose_debug(ee_start_pos, push_orn, label="ee_start")
        draw_pose_debug(ee_end_pos, push_orn, label="ee_end")

        MAX_JOINT_STEP  = 0.3
        MAX_RESTARTS    = 5
        N_STEPS         = 20
        MAX_SUBDIV      = 4
        MAX_SEED_TRIES  = 5

        def greedy_cartesian_path(positions, push_orn, seed_conf, max_step=MAX_JOINT_STEP, max_subdivisions=MAX_SUBDIV, max_seed_tries=MAX_SEED_TRIES):

            # FINDING CANDIDATE CONFIGS FOR THE NEXT POINT
            def solve_segment(start_pos, end_pos, start_conf, attempt):
                tform = pb_robot.geometry.tform_from_pose((tuple(end_pos), push_orn))

                for i in range(max_seed_tries):
                    seed = start_conf if i == 0 else (
                        numpy.array(start_conf) + numpy.random.normal(0, 0.03, size=len(start_conf))
                    )
                    candidate = arm.ComputeIK(tform, seed_q=seed)
                    if candidate is None:
                        continue
                    diff = numpy.max(numpy.abs(numpy.array(candidate) - numpy.array(start_conf)))
                    if diff > max_step:
                        continue
                    if not arm.IsCollisionFree(candidate, obstacles=fixed):
                        continue

                    return [candidate]
                
                # IF NO CANDIDATE FOUND, SUBDIVIDE
                # check if already too many subivisions
                if attempt > max_subdivisions:
                    return None
                
                mid = (numpy.array(start_pos) + numpy.array(end_pos)) / 2
                first_half = solve_segment(start_pos, mid, start_conf, attempt + 1)
                if first_half is None:
                    return None
                second_half = solve_segment(mid, end_pos, first_half[-1], attempt + 1)
                if second_half is None:
                    return None
                return first_half + second_half
        
            path = []
            current_conf = seed_conf
            current_pos = approach_pos
            for pos in positions:
                segment = solve_segment(current_pos, pos, current_conf, attempt=0)
                if segment is None:
                    return None
                path.extend(segment)
                current_conf = segment[-1]
                current_pos = pos

            return path

       # PART 1 APPROACH
        approach_tform = pb_robot.geometry.tform_from_pose(
            (tuple(approach_pos), push_orn)
        )
        path_to_approach = None

        for i in range(10):
            seed = conf1 if i == 0 else (
                numpy.array(conf1) + numpy.random.normal(0, 0.03, size=len(conf1))
            )
            approach_conf = arm.ComputeIK(approach_tform, seed_q=seed)
            if approach_conf is None:
                print(f"  attempt {i}: IK failed")
                continue
            if not arm.IsCollisionFree(approach_conf, obstacles=fixed):
                print(f"  attempt {i}: IK succeeded but in collision")
                continue

            path_to_approach = arm.birrt.PlanToConfiguration(
                arm, conf1, approach_conf, obstacles=fixed
            )
            if path_to_approach is None:
                if DEBUG_FAILURE: input('motion failed')
                print(f"  attempt {i}: IK+collision OK, birrt failed to connect")
                continue
            break
            
        if path_to_approach is None:
                if DEBUG_FAILURE: input('motion failed')
                return None

        # PART 2 PUSH
        push_positions = [
            approach_pos + t * (ee_end_pos - approach_pos)
            for t in numpy.linspace(0, 1, N_STEPS + 2)
        ]

        for attempt in range(MAX_RESTARTS):

            push_path = greedy_cartesian_path(
                push_positions[1:],
                push_orn,
                seed_conf=approach_conf,
                max_step=MAX_JOINT_STEP
            )

            if push_path is None:
                print(f"  Greedy path failed on attempt {attempt+1}, restarting...")

            command = [
                pb_robot.vobj.JointSpacePath(arm, path_to_approach),
                pb_robot.vobj.JointSpacePath(arm, push_path),
            ]
            print(f"  Push path found on attempt {attempt+1}")
            return (command,)

        # All restarts exhausted
        if DEBUG_FAILURE: input('Push planning failed after all restarts')
        return None

    return fn


# COLLISION CHECKERS
def pose_collision_test(o1, p1, o2, p2):
    '''Check if object o1 (at pose p1) is in collision with object o2 (at pose p2)'''
    o1_og = o1.get_transform()
    o2_og = o2.get_transform()

    o1.set_transform(p1.pose)
    o2.set_transform(p2.pose)
    collision = pb_robot.collisions.pairwise_collision(o1, o2)

    o1.set_transform(o1_og)
    o2.set_transform(o2_og)
    return not collision

def traj_collision_test(arm, traj, obj, pose):
    '''Tries to certify if collision-free. Return false if not collision free'''
    obj_og = obj.get_transform()
    obj.set_transform(pose.pose)

    for q in (traj[0]).path:
        arm.SetJointValues(q)
        inCollision = not arm.IsCollisionFree(q)
        if inCollision:
            obj.set_transform(obj_og)
            return False

    obj.set_transform(obj_og)
    return True