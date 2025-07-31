# import numpy as np
# import scipy.integrate as integrate
# from scipy.integrate import cumulative_trapezoid
from utilities import *

def plot_angles(time_array,
                folder_path,
                qw, 
                qx, 
                qy, 
                qz, 
                omega_x_rad, 
                omega_y_rad, 
                omega_z_rad,
                start_time,
                end_time,
                duration,
                stabilization_period):

    # getting roll pitch yaw
    rpy_rad = []
    quat = np.column_stack((qw,qx,qy,qz)) #this format feels dumb and redundant but idc lol i'm not CS
    for q in quat:
        un_norm = np.array([q[0], q[1], q[2], q[3]])
        quat_obj = Quaternion(un_norm/np.linalg.norm(un_norm))
        rpy = RollPitchYaw(quat_obj).vector()
        rpy_rad.append(rpy)
    rpy_rad = np.array(rpy_rad)

    rolls = rpy_rad[:, 0]
    pitches = rpy_rad[:, 1]
    yaws = rpy_rad[:, 2]

    rolls_degrees = np.degrees(rolls)
    pitches_degrees = np.degrees(pitches)
    yaws_degrees = np.degrees(yaws)

    # plot roll, pitch, yaw
    plt.figure()
    plt.plot(time_array,rolls_degrees)
    plt.title("Roll")
    plt.xlabel("Time [s]")
    plt.ylabel("Roll [degrees]")
    plt.savefig(f"{folder_path}/roll_v_time.png") 
    #plt.show()
    plt.close()
    # get roll amplitude
    if stabilization_period >= duration:
        avg_roll_amp, max_stable_roll_amp = get_amplitude(rolls_degrees, "roll", stabilization_period = stabilization_period)
    else:
        avg_roll_amp = 0
        max_stable_roll_amp = 0
    print(f"Max roll achieved: {np.max(np.abs(rolls_degrees))} degrees")

    plt.figure()
    plt.plot(time_array,pitches_degrees)
    plt.title("Pitch")
    plt.ylabel("Pitch [degrees]")
    plt.xlabel("Time [s]")
    plt.savefig(f"{folder_path}/pitch_v_time.png") 
    #plt.show()
    plt.close()
    # get pitch amplitude
    if duration >= stabilization_period:
        avg_pitch_amp, max_stable_pitch_amp = get_amplitude(pitches_degrees, "pitch", stabilization_period = stabilization_period)
    else:
        avg_pitch_amp = 0
        max_stable_pitch_amp = 0
        
    print(f"Max pitch achieved: {np.max(np.abs(pitches_degrees))} degrees")

    plt.figure()
    plt.plot(time_array,yaws_degrees)
    plt.title("Yaw")
    plt.ylabel("Yaw [degrees]")
    plt.xlabel("Time [s]")
    plt.savefig(f"{folder_path}/yaw_v_time.png") 
    #plt.show()
    plt.close()
    # get yaw amplitude
    avg_yaw_amp, max_stable_yaw_amp = get_amplitude(yaws_degrees, "yaw", stabilization_period = stabilization_period)
    print(f"Max yaw achieved: {np.max(np.abs(yaws_degrees))} degrees")


    # Roll vs yaw
    plt.figure()
    plt.plot(rolls_degrees[:stabilization_period],yaws_degrees[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(rolls_degrees[stabilization_period:],yaws_degrees[stabilization_period:], label="Stable Gait", color='r')
    plt.plot(rolls_degrees[0],yaws_degrees[0], 'o', color='purple', linestyle='none', label="Start point") # start point
    plt.plot(rolls_degrees[-1],yaws_degrees[-1], 'bD', linestyle='none', label="End point") # end point
    plt.title("Roll vs Yaw")
    plt.ylabel("Yaw [degrees]")
    plt.xlabel("Roll [degrees]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/roll_v_yaw.png") 
    #plt.show()
    plt.close()


    # 3D plot (method 2)
    rvy_points = np.array([rolls_degrees, yaws_degrees, time_array]).T.reshape(-1, 1, 3)
    rvy_segments = np.concatenate([rvy_points[:-1], rvy_points[1:]], axis=1)
    rvy_norm = Normalize(vmin=time_array.min(), vmax=time_array.max())
    rvy_colors = plt.cm.viridis(rvy_norm(time_array))
    rvy_fig2 = plt.figure()
    rvy_ax2 = rvy_fig2.add_subplot(111, projection='3d')
    rvy_lc = Line3DCollection(rvy_segments, cmap='viridis', norm=rvy_norm)
    rvy_lc.set_array(time_array)
    rvy_ax2.add_collection(rvy_lc)
    rvy_ax2.set_zlabel('Time [s]')
    rvy_ax2.set_xlabel('Roll [deegrees]')
    rvy_ax2.set_ylabel('Yaw [degrees]')
    rvy_ax2.set_title('Roll vs Yaw vs Time ')
    rvy_fig2.colorbar(rvy_lc, ax=rvy_ax2, label='Time')
    rvy_ax2.set_zlim(time_array.min(), time_array.max())
    rvy_ax2.set_xlim(rolls_degrees.min(), rolls_degrees.max())
    rvy_ax2.set_ylim(yaws_degrees.min(), yaws_degrees.max())
    plt.savefig(f"{folder_path}/roll_v_yaw3D.png") 
    #plt.show()
    plt.close()

    # roll vs pitch
    plt.figure()
    plt.plot(rolls_degrees[:stabilization_period],pitches_degrees[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(rolls_degrees[stabilization_period:],pitches_degrees[stabilization_period:], label="Stable Gait", color='r')
    plt.plot(rolls_degrees[0], pitches_degrees[0], 'o', color='purple', linestyle='none', label="Start point") # start point
    plt.plot(rolls_degrees[-1], pitches_degrees[-1], 'bD', linestyle='none', label="End point") # end point
    plt.title("Roll vs Pitch")
    plt.ylabel("Pitch [degrees]")
    plt.xlabel("Roll [degrees]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/roll_v_pitch.png") 
    #plt.show()
    plt.close()

    # 3D plot (method 2)
    rvp_points = np.array([rolls_degrees, pitches_degrees, time_array]).T.reshape(-1, 1, 3)
    rvp_segments = np.concatenate([rvp_points[:-1], rvp_points[1:]], axis=1)
    rvp_norm = Normalize(vmin=time_array.min(), vmax=time_array.max())
    rvp_colors = plt.cm.viridis(rvp_norm(time_array))
    rvp_fig2 = plt.figure()
    rvp_ax2 = rvp_fig2.add_subplot(111, projection='3d')
    rvp_lc = Line3DCollection(rvp_segments, cmap='viridis', norm=rvp_norm)
    rvp_lc.set_array(time_array)
    rvp_ax2.add_collection(rvp_lc)
    rvp_ax2.set_zlabel('Time [s]')
    rvp_ax2.set_xlabel('Roll [deegrees]')
    rvp_ax2.set_ylabel('Pitches [degrees]')
    rvp_ax2.set_title('Roll vs Pitch vs Time ')
    rvp_fig2.colorbar(rvp_lc, ax=rvp_ax2, label='Time')
    rvp_ax2.set_zlim(time_array.min(), time_array.max())
    rvp_ax2.set_xlim(rolls_degrees.min(), rolls_degrees.max())
    rvp_ax2.set_ylim(pitches_degrees.min(), pitches_degrees.max())
    plt.savefig(f"{folder_path}/roll_v_pitch3D.png") 
    #plt.show()
    plt.close()

    # 3D plot (method 2)
    rpy_points = np.array([rolls_degrees, pitches_degrees, yaws_degrees]).T.reshape(-1, 1, 3)
    rpy_segments = np.concatenate([rpy_points[:-1], rpy_points[1:]], axis=1)
    rpy_norm = Normalize(vmin=yaws_degrees.min(), vmax=yaws_degrees.max())
    rpy_colors = plt.cm.viridis(rpy_norm(yaws_degrees))
    rpy_fig2 = plt.figure()
    rpy_ax2 = rpy_fig2.add_subplot(111, projection='3d')
    rpy_lc = Line3DCollection(rpy_segments, cmap='viridis', norm=rpy_norm)
    rpy_lc.set_array(yaws_degrees)
    rpy_ax2.add_collection(rpy_lc)
    rpy_ax2.set_zlabel('Yaw [degrees]')
    rpy_ax2.set_xlabel('Roll [deegrees]')
    rpy_ax2.set_ylabel('Pitches [degrees]')
    rpy_ax2.set_title('Roll vs Pitch vs Yaw')
    rpy_fig2.colorbar(rpy_lc, ax=rpy_ax2, label='Yaw')
    rpy_ax2.set_zlim(yaws_degrees.min(), yaws_degrees.max())
    rpy_ax2.set_xlim(rolls_degrees.min(), rolls_degrees.max())
    rpy_ax2.set_ylim(pitches_degrees.min(), pitches_degrees.max())
    plt.savefig(f"{folder_path}/rpy3D.png") 
    #plt.show()
    plt.close()

    # 3D plot
    if duration >= end_time:
        # start_time = 16500  # time of start of new cycle after 10 seconds 
        # period = int(1 / freq * 1000)  # Convert period of sine wave to milliseconds (approx. 730 ms)
        # end_time = start_time + period  # End time for one full gait cycle
        rpy_points = np.array([rolls_degrees[start_time:end_time], pitches_degrees[start_time:end_time], yaws_degrees[start_time:end_time]]).T.reshape(-1, 1, 3)
        rpy_segments = np.concatenate([rpy_points[:-1], rpy_points[1:]], axis=1)
        rpy_norm = Normalize(vmin=yaws_degrees[start_time:end_time].min(), vmax=yaws_degrees[start_time:end_time].max())
        rpy_colors = plt.cm.viridis(rpy_norm(yaws_degrees[start_time:end_time]))
        rpy_fig2 = plt.figure()
        rpy_ax2 = rpy_fig2.add_subplot(111, projection='3d')
        rpy_lc = Line3DCollection(rpy_segments, cmap='viridis', norm=rpy_norm)
        rpy_lc.set_array(yaws_degrees[start_time:end_time])
        rpy_ax2.add_collection(rpy_lc)
        rpy_ax2.set_zlabel('Yaw [degrees]')
        rpy_ax2.set_xlabel('Roll [deegrees]')
        rpy_ax2.set_ylabel('Pitches [degrees]')
        rpy_ax2.set_title('Roll vs Pitch vs Yaw One Cycle')
        rpy_fig2.colorbar(rpy_lc, ax=rpy_ax2, label='Yaw')
        rpy_ax2.set_zlim(yaws_degrees[start_time:end_time].min(), yaws_degrees[start_time:end_time].max())
        rpy_ax2.set_xlim(rolls_degrees[start_time:end_time].min(), rolls_degrees[start_time:end_time].max())
        rpy_ax2.set_ylim(pitches_degrees[start_time:end_time].min(), pitches_degrees[start_time:end_time].max())
        plt.savefig(f"{folder_path}/rpy3D_onecycle.png") 
        #plt.show()
        plt.close()
    else:
        pass

    # convert angular velocites around xyz axes (euler angle rates) to roll pitch yaw rates and plo
    conversion_factor = 180 / np.pi # drake gives things in radians so we need to change it to degrees
    # omega_x = np.array(omega_x_rad) * conversion_factor
    # omega_y = np.array(omega_y_rad) * conversion_factor
    # omega_z = np.array(omega_z_rad) * conversion_factor
    omega_x = np.array(omega_x_rad)
    omega_y = np.array(omega_y_rad)
    omega_z = np.array(omega_z_rad)
    roll_rate, pitch_rate, yaw_rate = angular_velocity_to_rpy_rates(omega_x, omega_y, omega_z, rolls, pitches)

    # plot euler angle rates
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(time_array,omega_x, color='r')
    plt.title('Euler Anglular Velocty: $\\omega_x$')
    plt.ylabel('$\\omega_x$ [radians/s]')
    plt.xlabel('Time [s]')
    
    plt.subplot(3, 1, 2)
    plt.plot(time_array,omega_y, color='b')
    plt.title('Euler Anglular Velocty: $\\omega_y$')
    plt.ylabel('$\\omega_y$ [radians/s]')
    plt.xlabel('Time [ms]')

    plt.subplot(3, 1, 3)
    plt.plot(time_array,omega_z, color='g')
    plt.title('Euler Anglular Velocty: $\\omega_z$')
    plt.ylabel('$\\omega_z$ [radians/s]')
    plt.xlabel('Time [ms]')
    plt.tight_layout()
    plt.savefig(f"{folder_path}/euler_rates.png") 
    #plt.show()
    plt.close()

    # plot roll, pitch, yaw rate
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(time_array,roll_rate, color='r')
    plt.title("Roll Rate")
    plt.ylabel("Roll Rate [radians/s]")
    plt.xlabel("Time [ms]")

    plt.subplot(3, 1, 2)
    plt.plot(time_array,yaw_rate, color='b')
    plt.title("Yaw Rate")
    plt.ylabel("Yaw Rate [radians/s]")
    plt.xlabel("Time [s]")

    plt.subplot(3, 1, 3)
    plt.plot(time_array,pitch_rate, color='g')
    plt.title("Pitch Rate")
    plt.ylabel("Pitch Rate [radians/s]")
    plt.xlabel("Time [s]")
    plt.tight_layout()
    plt.savefig(f"{folder_path}/rpy_rates.png") 
    #plt.show()
    plt.close()
    return rolls, pitches, yaws, rolls_degrees, pitches_degrees, yaws_degrees, pitch_rate, roll_rate, yaw_rate, omega_x, omega_y, omega_z, avg_roll_amp, avg_pitch_amp, avg_yaw_amp 


# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_com_and_roc(
    folder_path, 
    com_per_link, 
    com_x, 
    com_y, 
    com_z, 
    com_vx,
    com_vy,
    com_vz,
    left_contact_points, 
    right_contact_points,
    dt,
    time_array,
    total_mass
    ):

    com_accx = np.diff(com_vx) / dt
    com_accy = np.diff(com_vy) / dt
    com_accz = np.diff(com_vz) / dt

    # plot COM
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(time_array, com_x)
    plt.ylabel("COM in X [m]")
    plt.xlabel("Time [s]")
    plt.title("Center of mass position over time")
    
    plt.subplot(3, 1, 2)
    plt.plot(time_array, com_y)
    plt.ylabel("COM in Y [m]")
    plt.xlabel("Time [s]")
    plt.title("Center of mass position over time")
    
    plt.subplot(3, 1, 3)
    plt.plot(time_array, com_z)
    plt.ylabel("COM in Z [m]")
    plt.xlabel("Time [s]")
    plt.title("Center of mass position over time")
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85), ncol=2, frameon=False)
    plt.tight_layout()
    plt.tight_layout()
    plt.savefig(f"{folder_path}/com_position.png") 
    #plt.show()
    plt.close()

    plt.figure()
    plt.plot(com_x, com_y)
    plt.ylabel("COM in Y [m]")
    plt.xlabel("COM in X [m]")
    plt.title("Center of mass position over time")
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/com_xy.png") 
    #plt.show()
    plt.close()


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.scatter3D(com_x, com_y, com_z, label="CoM Points", color="red", s = 2)
    ax.plot3D(com_x, com_y, com_z, label="CoM Trajectory", color="blue")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Center of Mass (CoM) Points")
    # ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.subplots_adjust(bottom=0.2)
    plt.savefig(f"{folder_path}/com_position.png", bbox_inches='tight', dpi=300) 
    #plt.show()

    # plot force
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(com_accx*total_mass)
    plt.ylabel("X Force")
    plt.xlabel("Time [ms]")
    plt.xlim(20000, 22000)
    plt.ylim(-5,10)
    plt.title("Center of mass - Force over time")
    
    plt.subplot(3, 1, 2)
    plt.plot(com_accy*total_mass)
    plt.ylabel("Y Force")
    plt.xlabel("Time [ms]")
    plt.xlim(20000, 22000)
    plt.ylim(-5,10)
    
    plt.subplot(3, 1, 3)
    plt.plot(com_accz*total_mass)
    plt.ylabel("Z Force")
    plt.xlabel("Time [ms]")
    plt.xlim(20000, 22000)
    plt.ylim(-5,10)
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85), ncol=2, frameon=False)

    plt.tight_layout()
    plt.savefig(f"{folder_path}/com_force.png") 
    #plt.show()
    plt.close()

    # plot acc
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(com_accx)
    plt.ylabel("X Acceleration")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    plt.title("Center of mass - Acceleration over time")
    
    plt.subplot(3, 1, 2)
    plt.plot(com_accy)
    plt.ylabel("Y Acceleration")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    
    plt.subplot(3, 1, 3)
    plt.plot(com_accz)
    plt.ylabel("Z Acceleration")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85), ncol=2, frameon=False)

    plt.tight_layout()
    plt.savefig(f"{folder_path}/com_acc.png") 
    #plt.show()
    plt.close()

    # plot vel
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(com_vx)
    plt.ylabel("X Velocity")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    plt.title("Center of mass - Velocity over time")
    
    plt.subplot(3, 1, 2)
    plt.plot(com_vy)
    plt.ylabel("Y Velocity")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    
    plt.subplot(3, 1, 3)
    plt.plot(com_vz)
    plt.ylabel("Z Velocity")
    plt.xlabel("Time [ms]")
    plt.ylim(-5,10)
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85), ncol=2, frameon=False)
    
    plt.tight_layout()
    plt.savefig(f"{folder_path}/com_vel.png") 
    #plt.show()
    plt.close()
    
    # Convert contact points to arrays
    left_contacts = np.array(left_contact_points)
    right_contacts = np.array(right_contact_points)
    
    # COM X vs Z
    fig = plt.figure()
    plt.plot(com_x, com_z, label='Whole Body COM', color='red')
    if 'left_roc_marker' in com_per_link:
        plt.plot(com_per_link['left_roc_marker'][:, 0], com_per_link['left_roc_marker'][:, 2], label='Left ROC Marker', color='orange')
    if 'right_roc_marker' in com_per_link:
        plt.plot(com_per_link['right_roc_marker'][:, 0], com_per_link['right_roc_marker'][:, 2], label='Right ROC Marker', color='purple')
    if len(left_contacts) > 0:
        plt.scatter(left_contacts[:, 0], left_contacts[:, 2], label='Left Foot Contact', marker='o', color='blue', s=10)
    if len(right_contacts) > 0:
        plt.scatter(right_contacts[:, 0], right_contacts[:, 2], label='Right Foot Contact', marker='x', color='green', s=10)
    plt.xlabel("X [m]")
    plt.ylabel("Z [m]")
    plt.title("COM + Contact Points (X vs Z)")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=2, frameon=False)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, "com_and_roc_xz.png"))
    #plt.show()
    plt.close(fig)

    # COM Y vs Z
    fig = plt.figure()
    plt.plot(com_y, com_z, label='Whole Body COM', color='red')
    if 'left_roc_marker' in com_per_link:
        plt.plot(com_per_link['left_roc_marker'][:, 1], com_per_link['left_roc_marker'][:, 2], label='Left ROC Marker', color='orange')
    if 'right_roc_marker' in com_per_link:
        plt.plot(com_per_link['right_roc_marker'][:, 1], com_per_link['right_roc_marker'][:, 2], label='Right ROC Marker', color='purple')
    if len(left_contacts) > 0:
        plt.scatter(left_contacts[:, 1], left_contacts[:, 2], label='Left Foot Contact', marker='o', color='blue', s=10)
    if len(right_contacts) > 0:
        plt.scatter(right_contacts[:, 1], right_contacts[:, 2], label='Right Foot Contact', marker='x', color='green', s=10)
    plt.xlabel("Y [m]")
    plt.ylabel("Z [m]")
    plt.title("COM + Contact Points (Y vs Z)")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=2, frameon=False)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, "com_and_roc_yz.png"))
    #plt.show()
    plt.close(fig)
    

    
def draw_slope_line(length=8.0, pitch=0.05, origin_x=1.05, origin_z=0.0, **kwargs):
    x_vals = np.linspace(origin_x - length / 2, origin_x + length / 2, 100)
    z_vals = origin_z + np.tan(pitch) * (x_vals - origin_x)
    plt.plot(x_vals, z_vals, label='Slope', color='gray', linestyle='--', linewidth=1.5, **kwargs)

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_controls(time_array,
                 folder_path,
                 hip_real_torque,
                 hip_real_angvel,
                 hip_real_angle
                 ):
    
    # plot hip angle
    plt.figure()
    plt.plot(time_array,hip_real_angle, label = "Real Angle")
    plt.plot
    plt.title("Hip Joint Angle")
    plt.ylabel("Joint Angle [degrees]")
    plt.xlabel("Time [s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_angle.png") 
    #plt.show()
    plt.close()

    # plot hip input angular velocity angle
    plt.figure()
    plt.plot(time_array, hip_real_angvel)
    plt.plot
    plt.title("Hip Angular Velocity")
    plt.ylabel("Angular Velocity [rad/s]")
    plt.xlabel("Time [s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_angvel.png") 
    #plt.show()
    plt.close()

    # plot hip control signal (torque input)
    plt.figure()
    plt.plot(time_array,hip_real_torque, label = "Real Torque")
    plt.plot
    plt.title("Hip Torque: Control Signal")
    plt.ylabel("Torque [Nm]")
    plt.xlabel("Time [s]")
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_input_torque.png") 
    #plt.show()
    plt.close()

    # plot hip angle - zoomed in
    plt.figure()
    plt.plot(time_array,hip_real_angle, label = "Real Angle")
    plt.plot
    plt.title("Hip Joint Angle")
    plt.ylabel("Joint Angle [degrees]")
    plt.xlabel("Time [s]")
    plt.xlim(5,7)
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_angle_zoomedin.png") 
    #plt.show()
    plt.close()

    # plot hip input angular velocity angle - zoomed in
    plt.figure()
    plt.plot(time_array, hip_real_angvel)
    plt.plot
    plt.title("Hip Angular Velocity")
    plt.ylabel("Angular Velocity [rad/s]")
    plt.xlabel("Time [s]")
    plt.xlim(5,7)
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_angvel_zoomedin.png") 
    #plt.show()
    plt.close()

    # plot hip control signal (torque input) - zoomed in
    plt.figure()
    plt.plot(time_array,hip_real_torque, label = "Real Torque")
    plt.plot
    plt.title("Hip Torque: Control Signal")
    plt.ylabel("Torque [Nm]")
    plt.xlabel("Time [s]")
    plt.xlim(5,7)
    # plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/hip_input_torque_zoomedin.png") 
    #plt.show()
    plt.close()

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_grf(time_array,
             folder_path, 
             left_contact_forces,
             right_contact_forces,
             left_contact_points,
             right_contact_points, 
             total_mass, 
             stabilization_period):
    
    norm_force = total_mass * 9.8

    left_contact_forces = np.array(left_contact_forces) 
    left_contact_points = np.array(left_contact_points)
    left_fx, left_fy, left_fz = left_contact_forces[:, 0], left_contact_forces[:, 1], left_contact_forces[:, 2]
    og_left_x, og_left_y, og_left_z = left_contact_points[:, 0], left_contact_points[:, 1], left_contact_points[:, 2]
    left_x = og_left_x[og_left_x != 0.000]
    left_y = og_left_y[og_left_y != 0.000]
    left_z = og_left_z[og_left_z != 0.000]


    right_contact_forces = np.array(right_contact_forces) 
    right_contact_points = np.array(right_contact_points)
    right_fx, right_fy, right_fz = right_contact_forces[:, 0], right_contact_forces[:, 1], right_contact_forces[:, 2]
    og_right_x, og_right_y, og_right_z = right_contact_points[:, 0], right_contact_points[:, 1], right_contact_points[:, 2]
    right_x = og_right_x[og_right_x != 0]
    right_y = og_right_y[og_right_y != 0]
    right_z = og_right_z[og_right_z != 0]

    total_left_grf = np.sqrt(left_fx**2 + left_fy**2 + left_fz**2)
    total_right_grf = np.sqrt(right_fx**2 + right_fy**2 + right_fz**2)

    # plot grf
    plt.figure()
    plt.plot(time_array, total_left_grf, label="Left Foot", color='r')
    plt.plot(time_array, total_right_grf, label="Right Foot", color='b')
    plt.ylabel("Total Ground Reaction Force [N]")
    plt.title("Contact Forces over Time")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/total_grf.png") 
    #plt.show()
    plt.close()


    # plot grf zoomed in 
    plt.figure()
    plt.plot(time_array, total_left_grf, label="Left Foot", color='r')
    plt.plot(time_array, total_right_grf, label="Right Foot", color='b')
    plt.ylabel("Total Ground Reaction Force [N]")
    plt.xlim(20,22)
    plt.title("Contact Forces over Time - Zoomed In")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/total_grf_zoomedin.png") 
    #plt.show()
    plt.close()


    # plot fx, fy, fz individually
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(time_array, -left_fx, label="Left Foot", color='r')
    plt.plot(time_array, -right_fx, label="Right Foot", color='b')
    plt.ylabel("Force in X")
    plt.title("Contact Forces over Time")
    
    plt.subplot(3, 1, 2)
    plt.plot(time_array, -left_fy, label="Left Foot", color='r')
    plt.plot(time_array, -right_fy, label="Right Foot", color='b')
    plt.ylabel("Force in Y")
    
    plt.subplot(3, 1, 3)
    plt.plot(time_array, -left_fz, label="Left Foot", color='r')
    plt.plot(time_array, -right_fz, label="Right Foot", color='b')
    plt.ylabel("Force in Z")
    plt.xlabel("Time Step")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85),ncol=2, frameon=False)
    
    plt.tight_layout()
    
    plt.savefig(f"{folder_path}/grf_fxfyfz.png") 
    #plt.show()
    plt.close()

    # plot fx, fy, fz individually zoomed in
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(time_array, -left_fx, label="Left Foot", color='r')
    plt.plot(time_array, -right_fx, label="Right Foot", color='b')
    plt.ylabel("Force in X")
    plt.xlim(20,22)
    plt.title("Contact Forces over Time Zoomed In")
    
    plt.subplot(3, 1, 2)
    plt.plot(time_array, -left_fy, label="Left Foot", color='r')
    plt.plot(time_array, -right_fy, label="Right Foot", color='b')
    plt.ylabel("Force in Y")
    plt.xlim(20,22)
    
    plt.subplot(3, 1, 3)
    plt.plot(time_array, -left_fz, label="Left Foot", color='r')
    plt.plot(time_array, -right_fz, label="Right Foot", color='b')
    plt.ylabel("Force in Z")
    plt.xlabel("Time Step")
    plt.xlim(20,22)
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.85),ncol=2, frameon=False)
    
    plt.tight_layout()
    plt.savefig(f"{folder_path}/ground_reaction_forces_zoomedin.png") 
    #plt.show()
    plt.close()

    # plot x, y, z contact points individually
    mask_x = og_left_x != 0.000
    filtered_time_array = time_array[mask_x]
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.scatter(filtered_time_array,left_x, label="Left Foot", color='r', s=1)
    mask_x = og_right_x != 0.000
    filtered_time_array = time_array[mask_x]
    plt.scatter(filtered_time_array,right_x, label="Right Foot", color='b', s=1)
    plt.ylabel("X [m] (Contact Points)")
    plt.xlabel("Times [s]")
    plt.title("Contact Points (Contact Patch Normal) over Time")
    
    plt.subplot(2, 1, 2)
    mask_y = og_left_y != 0.000
    filtered_time_array = time_array[mask_y]
    plt.scatter(filtered_time_array,left_y, label="Left Foot", color='r', s=1)
    mask_y = og_right_y != 0.000
    filtered_time_array = time_array[mask_y]
    plt.scatter(filtered_time_array,right_y, label="Right Foot", color='b', s=1)
    plt.ylabel("Y [m] (Contact Points)")
    plt.xlabel("Times [s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.55), ncol=2, frameon=False)
    plt.tight_layout()
    
    plt.tight_layout()
    plt.savefig(f"{folder_path}/contact_points.png") 
    #plt.show()
    plt.close()

    # plot x vs y (2D)
    plt.figure()
    plt.scatter(left_x, left_y, label="Left Foot", color='r', s=1)
    plt.scatter(right_x, right_y, label="Right Foot", color='b', s=1)
    plt.xlabel("X [m] (Contact Points)")
    plt.ylabel("Y [m] (Contact Points)")
    plt.title("Top View of Contact Patch Normal: X vs Y")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/contact_points_2D.png") 
    #plt.show()
    plt.close()

    return left_x, left_y, right_x, right_y, left_fx, left_fy, left_fz, right_fx, right_fy, right_fz 

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_link_coms(T, 
                   folder_path, 
                   com_per_link):

    # 3D Plot: All links together 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for name, coms in com_per_link.items():
        ax.plot3D(coms[:, 0], coms[:, 1], coms[:, 2], label=name)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("COM Trajectories (3D) - All Links")
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    fig.savefig(os.path.join(folder_path, "com_all_links_3d.png"))
    #plt.show()
    plt.close(fig)

    # 2D Plot: X vs Z (All links) 
    fig = plt.figure()
    for name, coms in com_per_link.items():
        plt.plot(coms[:, 0], coms[:, 2], label=name)
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.title("COM Trajectories (X vs Z) - All Links")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    fig.savefig(os.path.join(folder_path, "com_all_links_xz.png"))
    #plt.show()
    plt.close(fig)

    # 2D Plot: Y vs Z (All links)
    fig = plt.figure()
    for name, coms in com_per_link.items():
        plt.plot(coms[:, 1], coms[:, 2], label=name)
    plt.xlabel("Y")
    plt.ylabel("Z")
    plt.title("COM Trajectories (Y vs Z) - All Links")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    fig.savefig(os.path.join(folder_path, "com_all_links_yz.png"))
    #plt.show()
    plt.close(fig)

    # Per-link Plots 
    for name, coms in com_per_link.items():
        # 3D trajectory
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(coms[:, 0], coms[:, 1], coms[:, 2])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"{name} COM Trajectory (3D)")
        fig.savefig(os.path.join(folder_path, f"{name}_com_3d.png"))
        #plt.show()
        plt.close(fig)

        # 2D X vs Z
        fig = plt.figure()
        plt.plot(coms[:, 0], coms[:, 2])
        plt.xlabel("X")
        plt.ylabel("Z")
        plt.title(f"{name} COM Trajectory (X vs Z)")
        fig.savefig(os.path.join(folder_path, f"{name}_com_xz.png"))
        #plt.show()
        plt.close(fig)

        # 2D Y vs Z
        fig = plt.figure()
        plt.plot(coms[:, 1], coms[:, 2])
        plt.xlabel("Y")
        plt.ylabel("Z")
        plt.title(f"{name} COM Trajectory (Y vs Z)")
        fig.savefig(os.path.join(folder_path, f"{name}_com_yz.png"))
        #plt.show()
        plt.close(fig)
    
    # # Make animation/video of plots
    # generate_com_videos(com_per_link, T, folder_path=folder_path)

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_position_and_velocity(time_array,
                               folder_path,
                               x,
                               y,
                               z,
                               com_x,
                               com_y,
                               com_z,
                               xdot,
                               ydot,
                               zdot,
                               left_x,
                               left_y,
                               right_x,
                               right_y):
    # plot x
    plt.figure()
    plt.plot(x)
    plt.plot(x[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(len(x),x[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("X position of left leg")
    plt.ylabel("X [m] ")
    plt.xlabel("Time [ms]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/x_v_time.png") 
    #plt.show()
    plt.close()

    # plot y
    plt.figure()
    plt.plot(y)
    plt.plot(y[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(len(y),y[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Y position of left leg")
    plt.ylabel("Y [m]")
    plt.xlabel("Time [ms]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/y_v_time.png") 
    #plt.show()
    plt.close()


    # plot z
    plt.figure()
    plt.plot(z)
    plt.plot(z[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(len(z),z[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Z position of left leg")
    plt.ylabel("Z [m]")
    plt.xlabel("Time [s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/z_v_time.png") 
    #plt.show()
    plt.close()


    # plot x vs y
    plt.figure()
    plt.plot(x,y, color = 'orange')
    plt.plot(x[0], y[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(x[-1], y[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Top view of path: X vs Y")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/x_v_y.png") 
    #plt.show()
    plt.close()



    # plot x vs y path vs foot contact
    plt.figure()
    plt.plot(x,y, label = "Path of CoM", color = 'orange')
    plt.scatter(left_x, left_y, label="Left Foot Contact Points", color='r', s=1)
    plt.scatter(right_x, right_y, label="Right Foot Contact Points", color='b', s=1)
    plt.plot(x[0], y[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(x[-1], y[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Top view of path of base: X vs Y")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/x_v_y_pathvcontact.png") 
    #plt.show()
    plt.close()


    # plot y vs z
    plt.figure()
    plt.plot(y,z, color = 'orange')
    plt.plot(y[0], z[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(y[-1], z[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Top view of path: Y vs Z")
    plt.xlabel("Y [m]")
    plt.ylabel("Z [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/y_v_z.png") 
    #plt.show()
    plt.close()

    # plot x vs z
    plt.figure()
    plt.plot(x,z, color = 'orange')
    plt.plot(x[0], z[0], 'go', linestyle='none', label="Start") # start point
    plt.plot(x[-1], z[-1], 'rD', linestyle='none', label="End") # end point
    plt.plot
    plt.title("Top view of path: X vs Z")
    plt.xlabel("X [m]")
    plt.ylabel("Z [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/x_v_z.png") 
    #plt.show()
    plt.close()

    # plot xyz 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, c='r', label='Scatter Plot')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('3D Scatter Plot')
    ax.legend()
    plt.savefig(f"{folder_path}/com_xyz.png") 
    #plt.show()
    plt.close()

    # plot x speed
    plt.figure()
    plt.plot(time_array,xdot)
    plt.plot
    plt.title("Velocity (of left leg) in X-direction")
    plt.ylabel("Speed [m/s]")
    plt.xlabel("Time [s]")
    plt.savefig(f"{folder_path}/xdot_v_time.png") 
    #plt.show()
    plt.close()


    # plot y speed
    plt.figure()
    plt.plot(time_array,ydot)
    plt.plot
    plt.title("Velocity (of left leg) in Y-direction")
    plt.ylabel("Speed [m/s]")
    plt.xlabel("Time [s]")
    plt.savefig(f"{folder_path}/ydot_v_time.png") 
    #plt.show()
    plt.close()


    # plot z speed
    plt.figure()
    plt.plot(time_array,zdot)
    plt.plot
    plt.title("Velocity (of left leg) in Z-direction")
    plt.ylabel("Speed [m/s]")
    plt.xlabel("Time [s]")
    plt.savefig(f"{folder_path}/zdot_v_time.png") 
    #plt.show()
    plt.close()

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_power_and_energy(time_array,
                          folder_path,
                          hip_real_angvel,
                          hip_real_torque,
                          stabilization_period):
    
    # plot energy and power from hip    
    hip_real_torque = np.squeeze(hip_real_torque)     
    hip_real_angvel = np.squeeze(hip_real_angvel) 
    total_power = hip_real_torque * hip_real_angvel

    plt.figure()
    plt.plot(time_array,total_power, label="Total Mechanical Power")
    plt.title("Measured Power")
    plt.ylabel("Measured Power [W]")
    plt.xlabel("Time [s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/power.png") 
    #plt.show()
    plt.close()
    print(f"Max total power achieved: {np.max(np.abs(total_power[stabilization_period:]))} W")
    # print(f"Max power achieved after 10 seconds: {np.max(np.abs(total_powers[10000:]))} W")

    adj_time = time_array
    total_power = total_power.flatten()
    cumulative_energy = cumulative_trapezoid(total_power, adj_time, initial=0)
    energy_per_step = cumulative_energy[1:] - cumulative_energy[:-1]

    plt.figure()
    # plt.plot(adj_time[1:], energy_per_step, label = "Energy at Timestep")
    plt.plot(adj_time, cumulative_energy, label = "Cumulative Energy")
    plt.title("Measured Energy [J]")
    plt.xlabel("Time [s]") 
    plt.ylabel("")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/cumulative_energy.png") 
    #plt.show()
    plt.close()
    return total_power, cumulative_energy

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_stability_analysis(time_array,
                    folder_path,
                    start_time,
                    end_time,
                    duration,
                    stabilization_period,
                    pitches,
                    pitch_rate,
                    rolls,
                    roll_rate,
                    yaws,
                    yaw_rate,
                    x,
                    y,
                    z,
                    xdot,
                    ydot,
                    zdot
                    ):
    # stabilization_period = 10 * 1000

    # plot "limit cycle" (pitch)
    plt.figure()
    plt.plot(pitches[:stabilization_period],pitch_rate[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(pitches[stabilization_period:],pitch_rate[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Pitch vs Pitch rate")
    plt.xlabel("Pitch [radians]") 
    plt.ylabel("Pitch rate [radians/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_p_v_prate.png") 
    #plt.show()
    plt.close()


    # plot "limit cycle" 
    plt.figure()
    plt.plot(rolls[:stabilization_period],roll_rate[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(rolls[stabilization_period:],roll_rate[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Roll vs Roll rate")
    plt.xlabel("Roll [radians]") 
    plt.ylabel("Roll rate [radians/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_r_v_rrate.png") 
    #plt.show()
    plt.close()

    # plot "limit cycle" 
    plt.figure()
    plt.plot(yaws[:stabilization_period],yaw_rate[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(yaws[stabilization_period:],yaw_rate[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Yaw vs Yaw rate")
    plt.xlabel("Yaw [radians]") 
    plt.ylabel("Yaw rate [radians/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_y_v_yrate.png") 
    #plt.show()
    plt.close()

    # plot x vs x_dot 
    plt.figure()
    plt.plot(x[:stabilization_period],xdot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(x[stabilization_period:],xdot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("X position vs X velocity")
    plt.xlabel("X [m]") 
    plt.ylabel("X velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_x_v_xdot.png") 
    #plt.show()
    plt.close()

    # plot y vs y_dot 
    plt.figure()
    plt.plot(y[:stabilization_period],ydot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(y[stabilization_period:],ydot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Y position vs Y velocity")
    plt.xlabel("Y [m]") 
    plt.ylabel("Y velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_y_v_ydot.png") 
    #plt.show()
    plt.close()


    # plot z vs z_dot
    plt.figure()
    plt.plot(z[:stabilization_period],zdot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(z[stabilization_period:],zdot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Z position vs Z velocity")
    plt.xlabel("Z [m]") 
    plt.ylabel("Z velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_z_v_zdot.png") 
    #plt.show()
    plt.close()

    # plot x_dot vs z_dot
    plt.figure()
    plt.plot(xdot[:stabilization_period],zdot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(xdot[stabilization_period:],zdot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("X velocity vs Z velocity")
    plt.xlabel("X velocity [m/s]") 
    plt.ylabel("Z velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_xdot_v_zdot.png") 
    #plt.show()
    plt.close()

    # plot x vs z
    plt.figure()
    plt.plot(x[:stabilization_period],z[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(x[stabilization_period:],z[stabilization_period:], label="Stable Gait", color='r')
    plt.title("X Position vs Z Position")
    plt.xlabel("X [m]") 
    plt.ylabel("Z [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_x_v_z.png") 
    #plt.show()
    plt.close()

    # plot y_dot vs z_dot
    plt.figure()
    plt.plot(ydot[:stabilization_period],zdot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(ydot[stabilization_period:],zdot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Y velocity vs Z velocity")
    plt.xlabel("Y velocity [m/s]") 
    plt.ylabel("Z velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_ydot_v_zdot.png") 
    #plt.show()
    plt.close()

    # plot y vs z
    plt.figure()
    plt.plot(y[:stabilization_period],z[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(y[stabilization_period:],z[stabilization_period:], label="Stable Gait", color='r')
    plt.title("Y Position vs Z Position")
    plt.xlabel("Y [m]") 
    plt.ylabel("Z [m]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_y_v_z.png") 
    #plt.show()
    plt.close()

    # plot x_dot vs y_dot
    plt.figure()
    plt.plot(xdot[:stabilization_period],ydot[:stabilization_period], label="Gait Stabilization Period", color='g')
    plt.plot(xdot[stabilization_period:],ydot[stabilization_period:], label="Stable Gait", color='r')
    plt.title("X velocity vs Y velocity")
    plt.xlabel("X velocity [m/s]") 
    plt.ylabel("Y velocity [m/s]")
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
    plt.tight_layout()
    plt.savefig(f"{folder_path}/stability_xdot_v_ydot.png") 
    #plt.show()
    plt.close()

    # plot x vs y
    if duration >= stabilization_period:
        plt.figure()
        plt.plot(x[:stabilization_period],y[:stabilization_period], label="Gait Stabilization Period", color='g')
        plt.plot(x[stabilization_period:],y[stabilization_period:], label="Stable Gait", color='r')
        plt.title("X Position vs Y Position")
        plt.xlabel("X [m]") 
        plt.ylabel("Y [m]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_x_v_y.png") 
        #plt.show()
        plt.close()

    if duration >= end_time:
        # plot limit cycle for one cycle
        plt.figure()
        plt.plot(rolls[start_time:end_time],roll_rate[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Roll vs Roll rate for One Cycle")
        plt.xlabel("Roll [radians]") 
        plt.ylabel("Roll rate [radians/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_r_v_rrate_onecycle.png") 
        #plt.show()
        plt.close()

        # plot limit cycle
        plt.figure()
        plt.plot(yaws[start_time:end_time],yaw_rate[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Yaw vs Yaw rate for One Cycle")
        plt.xlabel("Yaw [radians]") 
        plt.ylabel("Yaw rate [radians/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_y_v_yrate_onecycle.png") 
        #plt.show()
        plt.close()

        # plot x vs x_dot for one cycle
        plt.figure()
        plt.plot(x[start_time:end_time],xdot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("X position vs X velocity for One Cycle")
        plt.xlabel("X [m]") 
        plt.ylabel("X velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_x_v_xdot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot y_dot vs z_dot for one cycle
        plt.figure()
        plt.plot(ydot[start_time:end_time],zdot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Y velocity vs Z velocity for One Cycle")
        plt.xlabel("Y velocity [m/s]") 
        plt.ylabel("Z velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_ydot_v_zdot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot y vs y_dot for one cycle
        plt.figure()
        plt.plot(y[start_time:end_time],ydot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Y position vs Y velocity for One Cycle")
        plt.xlabel("Y [m]") 
        plt.ylabel("Y velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_y_v_ydot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot z vs z_dot for one cycle
        plt.figure()
        plt.plot(z[start_time:end_time],zdot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Z position vs Z velocity for One Cycle")
        plt.xlabel("Z [m]") 
        plt.ylabel("Z velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_z_v_zdot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot x_dot vs z_dot for one cycle
        plt.figure()
        plt.plot(xdot[start_time:end_time],zdot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("X velocity vs Z velocity for One Cycle")
        plt.xlabel("X velocity [m/s]") 
        plt.ylabel("Z velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_xdot_v_zdot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot x vs z for one cycle
        plt.figure()
        plt.plot(x[start_time:end_time],z[start_time:end_time], label="Stable Gait", color='r')
        plt.title("X Position vs Z Position for One Cycle")
        plt.xlabel("X [m]") 
        plt.ylabel("Z [m]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_x_v_z_onecycle.png") 
        #plt.show()
        plt.close()

        # plot "limit cycle" (pitch) for one cycle
        plt.figure()
        plt.plot(pitches[start_time:end_time],pitch_rate[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Pitch vs Pitch rate for One Cycle")
        plt.xlabel("Pitch [radians]") 
        plt.ylabel("Pitch rate [radians/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_p_v_prate_onecycle.png") 
        #plt.show()
        plt.close()


        # plot x_dot vs y_dot for one cycle
        plt.figure()
        plt.plot(xdot[start_time:end_time],ydot[start_time:end_time], label="Stable Gait", color='r')
        plt.title("X velocity vs Y velocity for One Cycle")
        plt.xlabel("X velocity [m/s]") 
        plt.ylabel("Y velocity [m/s]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_xdot_v_ydot_onecycle.png") 
        #plt.show()
        plt.close()

        # plot y vs z for one cycle
        plt.figure()
        plt.plot(y[start_time:end_time],z[start_time:end_time], label="Stable Gait", color='r')
        plt.title("Y Position vs Z Position for One ycle")
        plt.xlabel("Y [m]") 
        plt.ylabel("Z [m]")
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
        plt.tight_layout()
        plt.savefig(f"{folder_path}/stability_y_v_z_onecycle.png") 
        #plt.show()
        plt.close()

        # plot x vs y for one cycle
        if duration >= end_time:
            plt.figure()
            plt.plot(x[start_time:end_time],y[start_time:end_time], label="Stable Gait", color='r')
            plt.title("X Position vs Y Position for One Cycle")
            plt.xlabel("X [m]") 
            plt.ylabel("Y [m]")
            plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)
            plt.tight_layout()
            plt.savefig(f"{folder_path}/stability_x_v_y_onecycle.png") 
            #plt.show()
            plt.close()
    else:
        pass
# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
