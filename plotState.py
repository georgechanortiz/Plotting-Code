import numpy as np
import matplotlib.pyplot as plt
from math import pi

def plotState(stateTraj, tWindow, *args):
    # Prepare environment
    


    # Use existing figure handles if requested
    if len(args) >= 3 and len(args[2]) > 0:
        COMTrajFig = args[2][0]
        linearStateFig = args[2][1]
        angularStateFig = args[2][2]
        jointPositionFig = args[2][3]
        jointVelocityFig = args[2][4]
        jointEffortFig = args[2][5]
        footPositionFig = args[2][6]
        footVelocityFig = args[2][7]
    else:
        h = plt.get_fignums()
        n = len(h)
        COMTrajFig = n + 1
        linearStateFig = n + 2
        angularStateFig = n + 3
        jointPositionFig = n + 4
        jointVelocityFig = n + 5
        jointEffortFig = n + 6
        footPositionFig = n + 7
        footVelocityFig = n + 8

    titles = args[1] if len(args) >= 2 else True
    lineStyle = args[0] if len(args) >= 1 else '-'

    # Calculate time indices
    if len(tWindow) == 0:
        traj_idx = np.arange(len(stateTraj["time"]))
    else:
        traj_idx = np.where(
            (stateTraj["time"] >= tWindow[0]) &
            (stateTraj["time"] <= tWindow[1])
        )[0]

    # Only use keys that exist in the dictionary, some keys are not included in reference trajectory
    keys_to_trim = ["time", "position", "velocity", "orientationRPY", "orientationQuat", "angularVelocity", "jointPosition", "jointVelocity", "jointEffort"]
    
    for key in keys_to_trim:
        if key in stateTraj and len(stateTraj[key]) > 0:
            stateTraj[key] = stateTraj[key][traj_idx]

    # Foot data 
    if "footPosition" in stateTraj:
        for i in range(len(stateTraj["footPosition"])):
            stateTraj["footPosition"][i] = stateTraj["footPosition"][i][traj_idx]
            stateTraj["footVelocity"][i] = stateTraj["footVelocity"][i][traj_idx]

    # Color definitions
    footColorVector = [np.array([166, 25, 46]) / 255, np.array([0, 45, 114]) / 255, np.array([0, 132, 61]) / 255, np.array([242, 169, 0]) / 255]
    bodyColorVector = footColorVector
    
    num_feet = 4
    jointColorVector = footColorVector

    # Plot 1: 3D Trajectory Plot
    fig1 = plt.figure(COMTrajFig)
    ax = plt.subplot(projection='3d')
    fig1.canvas.manager.set_window_title("body_and_foot_trajectories")

    
    if "position" in stateTraj:
        ax.plot(
            stateTraj["position"][:, 0],
            stateTraj["position"][:, 1],
            stateTraj["position"][:, 2],
            linestyle=lineStyle, label='Body'
        )
    
    if "footPosition" in stateTraj:
        for i in range(len(stateTraj["footPosition"])):
            ax.plot(
                stateTraj["footPosition"][i][:, 0],
                stateTraj["footPosition"][i][:, 1],
                stateTraj["footPosition"][i][:, 2],
                color=footColorVector[i % 4],
                linestyle=lineStyle
            )
            
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    if titles: 
        ax.set_title('Body and Foot Trajectories')
      
    # Plot 2: Linear States
    fig2 = plt.figure(linearStateFig, figsize=(12, 6))
    fig2.clf()
    fig2.canvas.manager.set_window_title("linear_states")
    
    ax1 = plt.subplot(2, 1, 1)
    if "position" in stateTraj and stateTraj["position"] is not None:
        for i in range(3):
            ax1.plot(stateTraj['time'], stateTraj['position'][:, i], 
                    color=bodyColorVector[i], linestyle=lineStyle)
    
    ax1.set_ylabel('Position (m)')
    if titles:
        ax1.set_title('Linear States')
    ax1.legend(['X', 'Y', 'Z'], loc='right')
    ax1.autoscale(enable=True, axis='both', tight=True)
    ax1.grid(True)

    # Second subplot - Velocity
    ax2 = plt.subplot(2, 1, 2)
    if "velocity" in stateTraj and stateTraj["velocity"] is not None:
        for i in range(3):
            ax2.plot(stateTraj['time'], stateTraj['velocity'][:, i], 
                    color=bodyColorVector[i], linestyle=lineStyle)
    
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.autoscale(enable=True, axis='both', tight=True)
    ax2.grid(True)

    # Align y-labels
    fig2.align_ylabels([ax1, ax2])
    plt.tight_layout()

    # Plot 3: Angular States
    if "orientationRPY" in stateTraj and stateTraj["orientationRPY"] is not None:
        fig3 = plt.figure(angularStateFig, figsize=(12, 6))
        fig3.clf()
        fig3.canvas.manager.set_window_title("angular_states")
        
        ax1 = plt.subplot(2, 1, 1)
        for i in range(3):
            ax1.plot(stateTraj['time'], stateTraj['orientationRPY'][:, i], 
                    color=bodyColorVector[i], linestyle=lineStyle)
        
        ax1.set_ylabel('Ang. Pos. (rad)')
        if titles:
            ax1.set_title('Angular States')
        ax1.legend(['Roll', 'Pitch', 'Yaw'], loc='right')
        ax1.set_ylim([-pi, pi])
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        # Second subplot - Angular Velocity
        ax2 = plt.subplot(2, 1, 2)
        if "angularVelocity" in stateTraj and stateTraj["angularVelocity"] is not None:
            for i in range(3):
                ax2.plot(stateTraj['time'], stateTraj['angularVelocity'][:, i], 
                        color=bodyColorVector[i], linestyle=lineStyle)
        
        ax2.set_ylabel('Ang. Vel. (rad/s)')
        ax2.set_xlabel('Time (s)')
        ax2.autoscale(enable=True, axis='both', tight=True)
        ax2.grid(True)

        fig3.align_ylabels([ax1, ax2])
        plt.tight_layout()

    # Plot 4: Joint Positions
    if "jointPosition" in stateTraj and stateTraj["jointPosition"] is not None:
        # Specify indices for leg joints 
        abIndex = [0, 3, 6, 9]
        hipIndex = [1, 4, 7, 10]
        kneeIndex = [2, 5, 8, 11]
        abadLim = [-1, 1]
        hipLim = [-pi/2, pi]
        kneeLim = [0, pi]

        fig4 = plt.figure(jointPositionFig, figsize=(12, 9))
        fig4.clf()
        fig4.canvas.manager.set_window_title("joint_positions")
        
        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(stateTraj['time'], stateTraj['jointPosition'][:, abIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax1.set_ylabel('Ab/Ad (rad)')
        if titles:
            ax1.set_title('Joint Angles')
        ax1.set_ylim(abadLim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(stateTraj['time'], stateTraj['jointPosition'][:, hipIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax2.set_ylabel('Hip (rad)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hipLim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(stateTraj['time'], stateTraj['jointPosition'][:, kneeIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax3.set_ylabel('Knee (rad)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(kneeLim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig4.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 5: Joint Velocities
    if "jointVelocity" in stateTraj and stateTraj["jointVelocity"] is not None:
        abadVelLim = [-37.7, 37.7]
        hipVelLim = [-37.7, 37.7]
        kneeVelLim = [-25, 25]

        fig5 = plt.figure(jointVelocityFig, figsize=(12, 9))
        fig5.clf()
        fig5.canvas.manager.set_window_title("joint_velocities")
        
        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(stateTraj['time'], stateTraj['jointVelocity'][:, abIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax1.set_ylabel('Ab/Ad (rad/s)')
        if titles:
            ax1.set_title('Joint Velocities')
        ax1.set_ylim(abadVelLim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(stateTraj['time'], stateTraj['jointVelocity'][:, hipIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax2.set_ylabel('Hip (rad/s)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hipVelLim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(stateTraj['time'], stateTraj['jointVelocity'][:, kneeIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax3.set_ylabel('Knee (rad/s)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(kneeVelLim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig5.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 6: Joint Effort
    if "jointEffort" in stateTraj and stateTraj["jointEffort"] is not None:
        abadEffLim = [-21, 21]
        hipEffLim = [-21, 21]
        kneeEffLim = [-32, 32]

        fig6 = plt.figure(jointEffortFig, figsize=(12, 9))
        fig6.clf()
        fig6.canvas.manager.set_window_title("joint_effort")
        
        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(stateTraj['time'], stateTraj['jointEffort'][:, abIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax1.set_ylabel('Ab/Ad (A)')
        if titles:
            ax1.set_title('Joint Effort')
        ax1.set_ylim(abadEffLim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(stateTraj['time'], stateTraj['jointEffort'][:, hipIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax2.set_ylabel('Hip (A)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hipEffLim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(stateTraj['time'], stateTraj['jointEffort'][:, kneeIndex[i]], 
                    color=jointColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax3.set_ylabel('Knee (A)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(kneeEffLim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig6.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 7: Foot Positions
    if "footPosition" in stateTraj and stateTraj["footPosition"]:
        fig7 = plt.figure(footPositionFig, figsize=(12, 9))
        fig7.clf()
        fig7.canvas.manager.set_window_title("foot_positions")
        
        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(stateTraj['time'], stateTraj['footPosition'][i][:, 0], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax1.set_ylabel('X (m)')
        if titles:
            ax1.set_title('Foot Positions')
        ax1.autoscale(enable=True, axis='both', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(stateTraj['time'], stateTraj['footPosition'][i][:, 1], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax2.set_ylabel('Y (m)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.autoscale(enable=True, axis='both', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(stateTraj['time'], stateTraj['footPosition'][i][:, 2], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax3.set_ylabel('Z (m)')
        ax3.set_xlabel('Time (s)')
        ax3.autoscale(enable=True, axis='both', tight=True)
        ax3.grid(True)

        fig7.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 8: Foot Velocities
    if "footVelocity" in stateTraj and stateTraj["footVelocity"]:
        fig8 = plt.figure(footVelocityFig, figsize=(12, 9))
        fig8.clf()
        fig8.canvas.manager.set_window_title("foot_velocities")
        
        # Calculate velocity axis limits
        vel_axis = 0
        for i in range(num_feet):
            vel_axis = max(vel_axis, np.max(np.abs(stateTraj['footVelocity'][i])))
        
        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(stateTraj['time'], stateTraj['footVelocity'][i][:, 0], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax1.set_ylabel('X (m/s)')
        if titles:
            ax1.set_title('Foot Velocities')
        ax1.set_ylim([-vel_axis, vel_axis])
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(stateTraj['time'], stateTraj['footVelocity'][i][:, 1], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax2.set_ylabel('Y (m/s)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim([-vel_axis, vel_axis])
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(stateTraj['time'], stateTraj['footVelocity'][i][:, 2], 
                    color=footColorVector[i], linewidth=2, linestyle=lineStyle)
        
        ax3.set_ylabel('Z (m/s)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim([-vel_axis, vel_axis])
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig8.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Return figure handles as numpy array
    figArray = np.array([COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, 
                        jointVelocityFig, jointEffortFig, footPositionFig, footVelocityFig])
    
    return figArray