from typing import List
import time

import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

from robo_algo.arm import RoboticArm, RoboticArmPlotter
from robo_algo.arm_controller import ArmController
from robo_algo.plotter_graphs import get_drawing2
from robo_algo.constants import *
import robo_algo.core as core
from robo_algo.core import Color


# Increase for debugging!
MAX_SPEED = np.deg2rad(1.0)

############################## YOUR CODE GOES HERE ####################################
#######################################################################################
# UA: Код для обчислення кінематики має бути тут               #
# EN: Your code for IK and drawing goes here.                                         #

def forward_kinematics(arm: RoboticArm, angles=None):
    if angles is None:
        angles = arm.get_angles()
    
    link_lengths = arm.link_lengths
    p0 = arm.p0
    
    cumsum_angles = np.cumsum(angles)
    
    end_x = p0[0] + np.sum(link_lengths * np.cos(cumsum_angles))
    end_y = p0[1] + np.sum(link_lengths * np.sin(cumsum_angles))
    
    return np.array([end_x, end_y])

def jacobian(link_lengths, link_angles):
    n = len(link_angles)
    jacobian_matrix = np.zeros((2, n))
    
    cumsum_angles = np.cumsum(link_angles)
    
    for j in range(n):
        jacobian_matrix[0, j] = -np.sum(link_lengths[j:] * np.sin(cumsum_angles[j:]))
        jacobian_matrix[1, j] = np.sum(link_lengths[j:] * np.cos(cumsum_angles[j:]))
    
    return jacobian_matrix

def inverse_kinematics(target_position, arm, max_iterations=50, tolerance=0.01, alpha=0.5):
    angles = arm.get_angles().copy()
    
    for iteration in range(max_iterations):
        current_pos = forward_kinematics(arm, angles)
        
        error = target_position - current_pos
        
        error_norm = np.linalg.norm(error)
        if error_norm < tolerance:
            break
        
        J = jacobian(arm.link_lengths, angles)
        
        J_pinv = np.linalg.pinv(J)
        
        delta_angles = alpha * (J_pinv @ error)
        
        angles = angles + delta_angles
    
    return angles

#######################################################################################
#######################################################################################


if __name__ == "__main__":
    ctx = core.RenderingContext("Task 2 - visualization")
    arm = RoboticArmPlotter(
        ctx,
        joint0_position=np.array([8, 8]),
        link_lengths=[5, 4, 3, 2],
        link_angles=[np.deg2rad(160), np.deg2rad(100), np.deg2rad(-40), np.deg2rad(-180)],
        thickness=0.1,
        color=Color(127, 127, 127, 255),
        joint_radius=0.3,
        joint_color=Color(200, 200, 200, 255),
    )
    arm.start_drawing()

    ### <UA>
    ### drawing1 це список масивів точок. Кожен масив описує фігуру для малювання.
    ### Кожна фігура малюється окремо. Не повинно бути ліній, які зʼєднують дві фігури.
    ### <EN>
    ### drawing1 is a list of arrays of points. Each array describes a shape to draw.
    ### Each shape must be plotted separately. No line must connect two shapes.
    ### Each shape is an array of points which must be plotted in order.
    drawing2: List[np.ndarray] = get_drawing2()

    running = True
    i_shape = 0
    i_point = 0
    
    num_line_steps = 10
    line_interpolation = []
    current_line_step = 0
    moving_to_start = False
    
    try:
        while running:
            # Check the event queue
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    # The user closed the window or pressed escape
                    running = False

            ctx.screen.fill((0, 0, 0, 0))
            arm.render()

            ############################## YOUR CODE GOES HERE ####################################
            #######################################################################################
            # UA: Код для обчислення зворотної кінематики та малювання має бути тут               #
            # EN: Your code for IK and drawing goes here.                                         #
            #######################################################################################
            #######################################################################################
            # Hint: see arm_controller interface, use `controller`
            # controller.move_to_angles([your predicted angles])
            drawing = drawing2

            if len(line_interpolation) == 0:
                if i_shape < len(drawing):
                    current_shape = drawing[i_shape]
                    
                    if i_point < len(current_shape):
                        target_pos = current_shape[i_point]
                        
                        if i_point == 0:
                            moving_to_start = True
                            arm.stop_drawing()
                            
                            target_angles = inverse_kinematics(target_pos, arm)
                            arm.set_angles(target_angles)
                            
                            arm.start_drawing()
                            moving_to_start = False
                            i_point += 1
                        else:
                            start_pos = current_shape[i_point - 1]
                            
                            line_interpolation = []
                            for alpha in np.linspace(0, 1, num_line_steps):
                                interp_pos = start_pos * (1 - alpha) + target_pos * alpha
                                line_interpolation.append(interp_pos)
                            
                            current_line_step = 0
                            i_point += 1
                    else:
                        arm.stop_drawing()
                        i_shape += 1
                        i_point = 0
            
            if len(line_interpolation) > 0 and not moving_to_start:
                interp_target = line_interpolation[current_line_step]
                
                target_angles = inverse_kinematics(interp_target, arm)
                
                arm.set_angles(target_angles)
                
                arm.draw()
                
                current_line_step += 1
                if current_line_step >= len(line_interpolation):
                    line_interpolation = []

            #######################################################################################
            #######################################################################################

            # Make Box2D simulate the physics of our world for one step.
            ctx.world.Step(TIME_STEP, 10, 10)
            # Flip the screen and try to keep at the target FPS
            pygame.display.flip()
            ctx.clock.tick(TARGET_FPS)
    except KeyboardInterrupt:
        print("Keyboard interrupt. Terminating...")
    pygame.quit()