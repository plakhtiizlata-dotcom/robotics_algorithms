from typing import List
import time

import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE, MOUSEBUTTONDOWN)

from robo_algo.arm import RoboticArm, RoboticArmPlotter
from robo_algo.arm_controller import ArmController
from robo_algo.constants import *
import robo_algo.core as core
from robo_algo.core import Color, ColorGreen


MAX_SPEED = np.deg2rad(1.5)


def forward_kinematics(arm: RoboticArm, angles=None):
    if angles is None:
        angles = arm.get_angles()
    
    base_position = arm.p0
    link_lengths = arm.link_lengths
    
    position = np.array(base_position, dtype=float)
    cumulative_angle = 0.0
    
    for i in range(len(angles)):
        cumulative_angle += angles[i]
        position[0] += link_lengths[i] * np.cos(cumulative_angle)
        position[1] += link_lengths[i] * np.sin(cumulative_angle)
    
    return position


def jacobian(link_lengths, link_angles):
    n = len(link_angles)
    J = np.zeros((2, n))
    
    cumulative_angles = np.cumsum(link_angles)
    
    for i in range(n):
        for j in range(i, n):
            J[0, i] += -link_lengths[j] * np.sin(cumulative_angles[j])
            J[1, i] += link_lengths[j] * np.cos(cumulative_angles[j])
    
    return J


def inverse_kinematics(target_position, arm, method='damped', max_iterations=15, tolerance=0.05):
    angles = arm.get_angles().copy()
    link_lengths = arm.link_lengths
    base_position = arm.p0
    
    max_reach = np.sum(link_lengths)
    target_distance = np.linalg.norm(target_position - base_position)
    
    clamped_target = target_position.copy()
    if target_distance > max_reach * 0.99:
        direction = (target_position - base_position) / target_distance
        clamped_target = base_position + direction * max_reach * 0.99
    
    for iteration in range(max_iterations):
        current_position = forward_kinematics(arm, angles)
        error = clamped_target - current_position
        error_magnitude = np.linalg.norm(error)
        
        if error_magnitude < tolerance:
            break
        
        J = jacobian(link_lengths, angles)
        
        if method == 'damped':
            lambda_sq = 0.005
            if target_distance > max_reach * 0.95:
                lambda_sq = 0.05
            JtJ = J.T @ J
            delta_angles = np.linalg.solve(JtJ + lambda_sq * np.eye(len(angles)), J.T @ error)
        elif method == 'pseudoinverse':
            J_pinv = np.linalg.pinv(J)
            delta_angles = J_pinv @ error
        elif method == 'transpose':
            alpha = 0.3
            delta_angles = alpha * J.T @ error
        else:
            lambda_sq = 0.005
            JtJ = J.T @ J
            delta_angles = np.linalg.solve(JtJ + lambda_sq * np.eye(len(angles)), J.T @ error)
        
        max_step = 0.5
        delta_norm = np.linalg.norm(delta_angles)
        if delta_norm > max_step:
            delta_angles = delta_angles * (max_step / delta_norm)
        
        angles = angles + delta_angles
    
    return angles


if __name__ == "__main__":
    ctx = core.RenderingContext("Task 4 - visualization")
    arm = RoboticArmPlotter(
        ctx,
        joint0_position=np.array([8, 8]),
        link_lengths=[2, 1, 1, 2, 1, 2],
        link_angles=[np.deg2rad(160), np.deg2rad(-80), np.deg2rad(130),
                     np.deg2rad(0), np.deg2rad(90), np.deg2rad(200)],
        thickness=0.1,
        color=Color(127, 127, 127, 255),
        joint_radius=0.3,
        joint_color=Color(200, 200, 200, 255),
    )
    controller = ArmController(arm, max_velocity=MAX_SPEED)

    pygame.font.init()
    my_font = pygame.font.SysFont(pygame.font.get_default_font(), 30)

    running = True
    target_point = np.array([5., 5.])
    start = 0
    try:
        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    running = False
                elif event.type == pygame.MOUSEMOTION:
                    target_point = core.from_pix(np.array(event.dict['pos']))
                elif event.type == MOUSEBUTTONDOWN:
                    target_point = core.from_pix(np.array(event.dict['pos']))

            ctx.screen.fill((0, 0, 0, 0))

            text_surface = my_font.render(
                f"Target: {target_point[0]:.2f}, {target_point[1]:.2f}",
                True, (255, 255, 255), (0,0,0))
            ctx.screen.blit(text_surface, (40, 40))
            pygame.draw.circle(ctx.screen, ColorGreen, center=core.to_pix(target_point), radius=15)

            if controller.is_idle():
                new_angles = inverse_kinematics(target_point, arm, method='damped')
                controller.move_to_angles(new_angles)
            else:
                controller.step()
            
            arm.render()

            ctx.world.Step(TIME_STEP, 10, 10)
            pygame.display.flip()
            ctx.clock.tick(TARGET_FPS)
    except KeyboardInterrupt:
        print("Keyboard interrupt. Terminating...")
    pygame.quit()