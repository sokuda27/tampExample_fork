#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Samplers needed for PDDLStream. Implements grasping, placing, planning, cutting and collision-checking
'''

import numpy
import pybullet as p
from . import pb_robot
import tampExample

DEBUG_FAILURE = False 

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
