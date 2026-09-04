import asyncio
import cv2
import airsim
import numpy as np
import time
import csv
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw,
                             VelocityBodyYawspeed, Attitude)

async def run_drone2_launch(client):
    vehicle_name = "Drone2"
    pose = airsim.Pose(airsim.Vector3r(-30, 10, 0), airsim.to_quaternion(0, 0, 0))
    if vehicle_name not in client.listVehicles():
        client.simAddVehicle(vehicle_name, "simpleflight", pose)
    client.enableApiControl(True, "Drone2")
    client.armDisarm(True, "Drone2")

    print("-- target armed")

    client.takeoffAsync(vehicle_name="Drone2")
    await asyncio.sleep(3)
    print("-- target takeoff complete")

    client.moveToZAsync(-25, 5, vehicle_name="Drone2")
    await asyncio.sleep(5)
    print("-- target reached altitude")
    await asyncio.sleep(2)
    client.moveByVelocityAsync(-5,5,-2,100,vehicle_name="Drone2")


async def run_px4_interception(client, drone, camera_name, image_type):
    print("-- Setting initial offboard setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0.0, -25.0, 180.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")
        await drone.action.disarm()
        return
    print("-- Offboard start")
    await asyncio.sleep(10)

    # --- logging setup ---
    log = {
        "t": [], "lam_ele": [], "lam_azi": [], "sig_ele": [], "sig_azi": [],
        "vel_n": [], "vel_e": [], "vel_d": [],
        "des_vel_n": [], "des_vel_e": [], "des_vel_d": []
    }
    t0 = time.time()
    log_path = "png_log.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["t", "lam_ele", "lam_azi", "sig_ele", "sig_azi",
                          "vel_n", "vel_e", "vel_d",
                          "des_vel_n", "des_vel_e", "des_vel_d"])




    # --- shared state updated by background telemetry tasks ---
    state = {"velocity": None, "quat": None}

    async def watch_velocity():
        async for velocity in drone.telemetry.velocity_ned():
            state["velocity"] = velocity

    async def watch_attitude():
        async for q in drone.telemetry.attitude_quaternion():
            state["quat"] = q

    vel_task = asyncio.create_task(watch_velocity())
    att_task = asyncio.create_task(watch_attitude())

    # wait until we have at least one reading of each before entering the loop
    while state["velocity"] is None or state["quat"] is None:
        await asyncio.sleep(0.05)

    u, v = None, None
    #wait until we have first detection of target to initialize line of sight LOS
    while u is None or v is None:
        u, v = detect_target(client, camera_name, image_type)
    print("-- Target detected")
    los = pixel_to_NED_LOS(client, u, v, 256, 144, 128, camera_name)
    lamda_0 = vec_to_angles(los)

    k = 3 #PNG constant
    sigma_offset = (1 - k) * lamda_0 + np.array([0.52, 0.0]) #constant between sigma and lambda graphs (proportional integration)
    try:
        while True:
            u, v = None, None
            while u is None or v is None:
                u, v = detect_target(client, camera_name, image_type)
                #calculate los from target pixel coordinate
            los = pixel_to_NED_LOS(client, u, v, 256, 144, 128, camera_name)
            #transform unit los to elevation and azimuth angles
            lamda = vec_to_angles(los)
            #calculation of sigma knowing sigma_dot = k * lambda_dot
            sigma = k * lamda + sigma_offset
            #transform sigma angles(azi and ele of desired vel unit vector) to desired vel unit vector nvd
            nvd = angles_to_vec(sigma)
            print("nvd=",nvd)

            velocity = state["velocity"]
            v_actual = np.sqrt(
                velocity.north_m_s ** 2 +
                velocity.east_m_s ** 2 +
                velocity.down_m_s ** 2
            )
            #saturate desired velocity vector magnitude
            vd = np.clip(v_actual + 0.4, 0, 25)

            desired_velocity = nvd * vd
            #proportional yaw controller to keep target at center of image
            w_psi = 0.01 * (u - 128)
            w_psi = (w_psi/np.pi)*180
            # --- log this iteration ---
            t = time.time() - t0
            log["t"].append(t)
            log["lam_ele"].append(lamda[0])
            log["lam_azi"].append(lamda[1])
            log["sig_ele"].append(sigma[0])
            log["sig_azi"].append(sigma[1])
            log["vel_n"].append(velocity.north_m_s)
            log["vel_e"].append(velocity.east_m_s)
            log["vel_d"].append(velocity.down_m_s)
            log["des_vel_n"].append(desired_velocity[0])
            log["des_vel_e"].append(desired_velocity[1])
            log["des_vel_d"].append(desired_velocity[2])
            log_writer.writerow([t, lamda[0], lamda[1], sigma[0], sigma[1],
                                  velocity.north_m_s, velocity.east_m_s, velocity.down_m_s,
                                  desired_velocity[0], desired_velocity[1], desired_velocity[2]])
            log_file.flush()
            q_msg = state["quat"]
            #px4 drone attitude
            q = np.array([q_msg.w, q_msg.x, q_msg.y, q_msg.z])
            #calculate rotation matrix from q
            R_w2b = quat_to_rot_world2local(q)
            desired_velocity_body = R_w2b @ desired_velocity
            #velocity in body since offboard mode offers yaw rate control only in case of cmd body velocity
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(desired_velocity_body[0],desired_velocity_body[1],
                                      desired_velocity_body[2], w_psi)
            )
            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("-- Program interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        vel_task.cancel()
        att_task.cancel()
        cv2.destroyAllWindows()
        log_file.close()
        plot_png_log(log)
        plot_velocity_log(log)

def plot_png_log(log):
    import matplotlib.pyplot as plt

    t = log["t"]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(t, np.degrees(log["lam_ele"]), label="lambda (LOS) elevation")
    axes[0].plot(t, np.degrees(log["sig_ele"]), label="sigma (cmd) elevation")
    axes[0].set_ylabel("Elevation (deg)")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, np.degrees(log["lam_azi"]), label="lambda (LOS) azimuth")
    axes[1].plot(t, np.degrees(log["sig_azi"]), label="sigma (cmd) azimuth")
    axes[1].set_ylabel("Azimuth (deg)")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend()
    axes[1].grid(True)

    fig.suptitle("Line-of-sight angle (\u03bb) vs Commanded velocity angle (\u03c3)")
    plt.tight_layout()
    plt.show()

def plot_velocity_log(log):
    import matplotlib.pyplot as plt

    t = log["t"]
    fig, ax = plt.subplots(figsize=(10, 6))

    # Actual velocity (solid lines)
    ax.plot(t, log["vel_n"], color="tab:blue", linestyle="-", label="v_north (actual)")
    ax.plot(t, log["vel_e"], color="tab:orange", linestyle="-", label="v_east (actual)")
    ax.plot(t, log["vel_d"], color="tab:green", linestyle="-", label="v_down (actual)")

    # Desired velocity (dashed lines), same color per axis
    ax.plot(t, log["des_vel_n"], color="tab:blue", linestyle="--", label="v_north (desired)")
    ax.plot(t, log["des_vel_e"], color="tab:orange", linestyle="--", label="v_east (desired)")
    ax.plot(t, log["des_vel_d"], color="tab:green", linestyle="--", label="v_down (desired)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Actual vs Desired Velocity (NED)")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()


def detect_target(client, camera_name, image_type):
    # ===== Object Detection =====
    rawImage = client.simGetImage(camera_name, image_type)
    u, v = None, None
    if rawImage:
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        detects = client.simGetDetections(camera_name, image_type)

        if detects:
            for detect in detects:
                # Draw bounding box
                cv2.rectangle(png,
                              (int(detect.box2D.min.x_val), int(detect.box2D.min.y_val)),
                              (int(detect.box2D.max.x_val), int(detect.box2D.max.y_val)),
                              (255, 0, 0), 2)
                u = (detect.box2D.min.x_val + detect.box2D.max.x_val) / 2
                v = (detect.box2D.min.y_val + detect.box2D.max.y_val) / 2
                cv2.putText(png, detect.name,
                            (int(detect.box2D.min.x_val), int(detect.box2D.min.y_val - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12))

        # Display the image
        cv2.imshow("AirSim - Object Detection", png)

        # Handle keyboard input for detection controls
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            client.simClearDetectionMeshNames(camera_name, image_type)
            print("-- Cleared detection filters")
        elif key == ord('a'):
            client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*")
            print("-- Added Cylinder detection filter")
    return u, v


def pixel_to_NED_LOS(client, u, v, image_width, image_height, f_pixels,
                     camera_name="0"):
    """
    Converts a pixel coordinate to a world-space ray (direction).

    Parameters:
        client         : connected airsim.MultirotorClient
        u, v           : pixel coordinates (0-based)
        image_width, height : image resolution in pixels
        f_pixels       : focal length in pixels (square pixels assumed)
        camera_name    : AirSim camera name, default "0"

    Returns:
        dir_world      : numpy unit vector [dx, dy, dz] in NED
    """
    # -- 1. Camera intrinsics --
    cx = image_width / 2.0
    cy = image_height / 2.0

    # Direction vector in camera frame (pixel units)
    los_cam = np.array([f_pixels, u - cx, v - cy], dtype=np.float64)
    los_cam /= np.linalg.norm(los_cam)  # unit length

    # -- 2. Get camera world pose directly --
    cam_info = client.simGetCameraInfo(camera_name)
    cam_pose = cam_info.pose  # airsim.Pose in NED

    # Quaternion (world->camera)
    q = [cam_pose.orientation.w_val,
         cam_pose.orientation.x_val,
         cam_pose.orientation.y_val,
         cam_pose.orientation.z_val]
    print(q)
    R_world2cam = quat_to_rot_world2local(q)
    R_cam2world = R_world2cam.T  # inverse = transpose for rotation matrices

    # -- 3. Transform direction to world NED --
    los_world = R_cam2world @ los_cam

    return los_world


def quat_to_rot_world2local(q):
    """
    q: array-like [w, x, y, z] (AirSim convention)
    Returns: 3x3 matrix R_w2l such that v_local = R_w2l @ v_world
    """
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    R = np.array([
        [1 - 2 * (q3 * q3 + q2 * q2), 2 * (q2 * q1 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
        [2 * (q2 * q1 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q3 * q2 + q0 * q1)],
        [2 * (q0 * q2 + q1 * q3), 2 * (q3 * q2 - q0 * q1), 1 - 2 * (q2 * q2 + q1 * q1)]
    ])
    return R


def vec_to_angles(v):
    x = v[0]
    y = v[1]
    z = v[2]

    elevation = np.arctan(z / np.sqrt(x ** 2 + y ** 2))
    azimuth = np.arctan2(y, x)

    q = np.array([elevation, azimuth], dtype=np.float64)
    return q


def angles_to_vec(a):
    ele = a[0]
    azi = a[1]
    x = np.cos(ele) * np.cos(azi)
    y = np.sin(azi) * np.cos(ele)
    z = np.sin(ele)
    vec = np.array([x, y, z], dtype=np.float64)
    return vec


async def main():
    """Main function that combines manual control and object detection"""
    # ===== AirSim Connection for Object Detection =====
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.reset()
    print("Waiting for AirSim...")
    client.confirmConnection()
    print("-- Connected to AirSim!")

    # ===== MAVSDK Connection for Manual Control =====
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14550")

    print("Waiting for px4 drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to px4 drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break
    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.set_takeoff_altitude(5.0)  # meters
    await drone.action.takeoff()
    # Set up object detection
    camera_name = "0"
    image_type = airsim.ImageType.Scene
    client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100) #detection distance in cm
    client.simAddDetectionFilterMeshName(camera_name, image_type, "Drone*") #detection mesh name
    print("-- Object detection configured for Drones")
    await asyncio.sleep(5)
    """Run both Drone2 and PX4 routines concurrently"""
    print("[Main] Starting both drones asynchronously...")
    await asyncio.gather(
        run_drone2_launch(client),
        run_px4_interception(client, drone, camera_name, image_type)
    )
    print("[Main] Both drones completed.")


if __name__ == "__main__":
    asyncio.run(main())
