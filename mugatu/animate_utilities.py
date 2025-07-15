def animate_individual_links(com_per_link, T, folder_path):
    os.makedirs(folder_path, exist_ok=True)

    for name, coms in com_per_link.items():
        num_frames = len(coms)
        # step = int(1.0 / T) 
        # frames = list(range(0, num_frames, step))
        # fps = 1

        # frames = list(range(num_frames))  
        # step = T * 1000  
        # fps = int(1/T)

        step = int(0.01 / T)
        interval = T * step * 1000  
        frames = range(0, num_frames, step)
        fps = int(1 / (T * step)) 

        # X vs Z
        fig_xz, ax_xz = plt.subplots()
        ax_xz.set_title(f"{name} COM X vs Z")
        ax_xz.set_xlabel("X")
        ax_xz.set_ylabel("Z")
        ax_xz.set_xlim(-0.2, 0.2)
        ax_xz.set_ylim(0, 0.3)
        line, = ax_xz.plot([], [], label="Trajectory")
        ax_xz.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)

        def update_xz(frame):
            line.set_data(coms[:frame+1, 0], coms[:frame+1, 2])
            return line,

        ani = animation.FuncAnimation(fig_xz, update_xz, frames=frames, interval=interval, blit=True)
        ani.save(os.path.join(folder_path, f"{name}_com_xz.mp4"), writer='ffmpeg', fps=fps, dpi=100)
        plt.close(fig_xz)

        # Y vs Z
        fig_yz, ax_yz = plt.subplots()
        ax_yz.set_title(f"{name} COM Y vs Z")
        ax_yz.set_xlabel("Y")
        ax_yz.set_ylabel("Z")
        ax_yz.set_xlim(-0.2, 0.2)
        ax_yz.set_ylim(0, 0.3)
        line_yz, = ax_yz.plot([], [], label="Trajectory")
        ax_yz.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)

        def update_yz(frame):
            line_yz.set_data(coms[:frame+1, 1], coms[:frame+1, 2])
            return line_yz,

        ani_yz = animation.FuncAnimation(fig_yz, update_yz, frames=frames, interval=interval, blit=True)
        ani_yz.save(os.path.join(folder_path, f"{name}_com_yz.mp4"), writer='ffmpeg', fps=fps, dpi=100)
        plt.close(fig_yz)

        # 3D
        fig_3d = plt.figure()
        ax_3d = fig_3d.add_subplot(111, projection='3d')
        ax_3d.set_title(f"{name} COM 3D")
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")
        ax_3d.set_xlim(-0.2, 0.2)
        ax_3d.set_ylim(-0.2, 0.2)
        ax_3d.set_zlim(0, 0.3)
        line_3d, = ax_3d.plot([], [], [], label="Trajectory")
        ax_3d.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)

        def update_3d(frame):
            xs = coms[:frame+1, 0]
            ys = coms[:frame+1, 1]
            zs = coms[:frame+1, 2]
            line_3d.set_data(xs, ys)
            line_3d.set_3d_properties(zs)
            return line_3d,

        ani_3d = animation.FuncAnimation(fig_3d, update_3d, frames=frames, interval=interval, blit=False)
        ani_3d.save(os.path.join(folder_path, f"{name}_com_3d.mp4"), writer='ffmpeg', fps=fps, dpi=100)
        plt.close(fig_3d)

def animate_com_2d_and_3d(com_per_link, T, folder_path):
    os.makedirs(folder_path, exist_ok=True)
    num_frames = min(len(coms) for coms in com_per_link.values())
    # step = int(1.0 / T)  
    # frames = list(range(0, num_frames, step))

    # frames = list(range(num_frames))  
    # step = T * 1000  
    # fps = int(1/T)

    step = int(0.01 / T)
    frames = range(0, num_frames, step)
    interval = T * step * 1000  
    fps = int(1 / (T * step)) 
    
    # X vs Z
    fig_xz, ax_xz = plt.subplots()
    ax_xz.set_xlabel("X")
    ax_xz.set_ylabel("Z")
    ax_xz.set_title("COM Animation (X vs Z)")
    lines_xz = {name: ax_xz.plot([], [], label=name)[0] for name in com_per_link}
    ax_xz.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    ax_xz.set_xlim(-0.2, 0.2)
    ax_xz.set_ylim(0, 0.3)

    def update_xz(frame):
        for name, line in lines_xz.items():
            coms = com_per_link[name]
            line.set_data(coms[:frame+1, 0], coms[:frame+1, 2])
        return lines_xz.values()

    ani_xz = animation.FuncAnimation(fig_xz, update_xz, frames=frames, interval=interval, blit=True)
    ani_xz.save(os.path.join(folder_path, "com_animation_xz.mp4"), writer='ffmpeg', fps=fps, dpi=100)
    plt.close(fig_xz)

    # Y vs Z
    fig_yz, ax_yz = plt.subplots()
    ax_yz.set_xlabel("Y")
    ax_yz.set_ylabel("Z")
    ax_yz.set_title("COM Animation (Y vs Z)")
    lines_yz = {name: ax_yz.plot([], [], label=name)[0] for name in com_per_link}
    ax_yz.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    ax_yz.set_xlim(-0.2, 0.2)
    ax_yz.set_ylim(0, 0.3)

    def update_yz(frame):
        for name, line in lines_yz.items():
            coms = com_per_link[name]
            line.set_data(coms[:frame+1, 1], coms[:frame+1, 2])
        return lines_yz.values()

    ani_yz = animation.FuncAnimation(fig_yz, update_yz, frames=frames, interval=interval, blit=True)
    ani_yz.save(os.path.join(folder_path, "com_animation_yz.mp4"), writer='ffmpeg', fps=fps, dpi=100)
    plt.close(fig_yz)

    # 3D
    fig_3d = plt.figure()
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.set_title("COM Animation (3D)")
    lines_3d = {name: ax_3d.plot([], [], [], label=name)[0] for name in com_per_link}
    ax_3d.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=2, frameon=False)
    ax_3d.set_xlim(-0.2, 0.2)
    ax_3d.set_ylim(-0.2, 0.2)
    ax_3d.set_zlim(0, 0.3)

    def update_3d(frame):
        for name, line in lines_3d.items():
            coms = com_per_link[name]
            line.set_data(coms[:frame+1, 0], coms[:frame+1, 1])
            line.set_3d_properties(coms[:frame+1, 2])
        return lines_3d.values()

    ani_3d = animation.FuncAnimation(fig_3d, update_3d, frames=frames, interval=interval, blit=False)
    ani_3d.save(os.path.join(folder_path, "com_animation_3d.mp4"), writer='ffmpeg', fps=fps, dpi=100)
    plt.close(fig_3d)


def generate_com_videos(com_per_link, T, folder_path):
    os.makedirs(folder_path, exist_ok=True)

    # per-link animations
    for name, coms in com_per_link.items():
        video_path = os.path.join(folder_path, "link_videos", name)
        print(f"Started making videos for {name}...")
        animate_individual_links({name: coms}, T, video_path)
        print(f"Finished making videos for {name}.")

    # combined animations
    all_links_path = os.path.join(folder_path, "all_links")
    print("Started making all-links videos...")
    animate_com_2d_and_3d(com_per_link, T, all_links_path)
    print("Finished making all-links videos.")

