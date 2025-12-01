import pyvista as pv
import numpy as np
import requests
from PIL import Image
from io import BytesIO
from pathlib import Path


def download_satellite_imagery(lat, lon, size_meters, zoom=19):
    try:
        # Calculate tile coordinates from lat/lon
        n = 2.0 ** zoom

        # Convert to tile coordinates
        lat_rad = np.radians(lat)
        xtile = int((lon + 180.0) / 360.0 * n)
        ytile = int((1.0 - np.log(np.tan(lat_rad) + (1 / np.cos(lat_rad))) / np.pi) / 2.0 * n)

        # Calculate resolution at this latitude and zoom level
        meters_per_pixel = 156543.03392 * np.cos(lat_rad) / (2 ** zoom)
        max_tiles = int(np.ceil((size_meters / meters_per_pixel) / 256))
        tiles_needed = max(3, max_tiles | 1)  # force odd number, symmetric

        print(f"Downloading imagery: {tiles_needed}x{tiles_needed} tiles at zoom {zoom}")
        print(f"Resolution: ~{meters_per_pixel:.2f} meters/pixel")

        # Download tiles
        tiles = []
        half_tiles = tiles_needed // 2

        for dy in range(-half_tiles, half_tiles + 1):
            row = []
            for dx in range(-half_tiles, half_tiles + 1):
                tile_x = xtile + dx
                tile_y = ytile + dy

                # ArcGIS World Imagery high-res service
                url = f"https://server.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/tile/{zoom}/{tile_y}/{tile_x}"

                try:
                    response = requests.get(url, timeout=10)
                    if response.status_code == 200:
                        img = Image.open(BytesIO(response.content))
                        row.append(img)
                    else:
                        # Create blank tile if download fails
                        row.append(Image.new('RGB', (256, 256), color='lightgreen'))
                except Exception as e:
                    print(f"Failed to download tile ({tile_x}, {tile_y}): {e}")
                    row.append(Image.new('RGB', (256, 256), color='lightgreen'))

            tiles.append(row)

        # Stitch tiles together
        tile_width = tiles[0][0].width
        tile_height = tiles[0][0].height

        full_width = len(tiles[0]) * tile_width
        full_height = len(tiles) * tile_height

        full_image = Image.new('RGB', (full_width, full_height))

        for i, row in enumerate(tiles):
            for j, tile in enumerate(row):
                full_image.paste(tile, (j * tile_width, i * tile_height))

        # Convert PIL image to numpy array for PyVista
        img_array = np.array(full_image)

        # Calculate actual coverage in meters
        actual_coverage = full_width * meters_per_pixel

        # Create PyVista texture
        texture = pv.Texture(img_array)

        print(f"Satellite imagery downloaded successfully ({full_width}x{full_height} px)")
        print(f"Actual ground coverage: ~{actual_coverage:.1f}m x {actual_coverage:.1f}m")

        return texture, actual_coverage

    except Exception as e:
        print(f"Failed to download satellite imagery: {e}")
        print("Using plain ground texture instead.")
        return None, size_meters

def quaternion_to_rotation_matrix(e0, e1, e2, e3):
    # Normalize quaternion
    norm = np.sqrt(e0 ** 2 + e1 ** 2 + e2 ** 2 + e3 ** 2)
    if norm > 0:
        e0, e1, e2, e3 = e0 / norm, e1 / norm, e2 / norm, e3 / norm

    # Convert to rotation matrix
    R = np.array([
        [1 - 2 * (e2 ** 2 + e3 ** 2), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2)],
        [2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 ** 2 + e3 ** 2), 2 * (e2 * e3 - e0 * e1)],
        [2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 ** 2 + e2 ** 2)]
    ])

    return R

def animate_rocket_flight(
        rocket_body_path,
        rocket_black_path=None,
        airbrake_leaf_paths=None,
        flight=None,
        controller=None,
        csv_path=None,
        csv_time_col='time',
        csv_altitude_col='altitude',
        csv_deployment_col='deployment',
        csv_velocity_col='velocity',
        fps=30,
        duration=None,
        output_path='flight.mp4',
        center_of_mass=(0, 0, 0.45),
        scale=0.001,
        launch_site_lat=38.9591829,
        launch_site_lon=-92.0998801,
        ground_size=8000,
        imagery_zoom=17,
        airbrake_deploy_func=None,
        airbrake_max_extension=0.05,
        airbrake_positions=None,
        camera_option=1
):

    # Load rocket body
    rocket_body = pv.read(rocket_body_path)
    rocket_body.points *= scale
    com = np.array(center_of_mass)
    rocket_body.points -= com
    original_body_points = rocket_body.points.copy()

    # Calculate the middle of the rocket in z-direction for camera focal point
    rocket_z_min = original_body_points[:, 2].min()
    rocket_z_max = original_body_points[:, 2].max()
    rocket_z_middle = (rocket_z_min + rocket_z_max) / 2

    # Load black components if provided
    rocket_black = None
    original_black_points = None
    if rocket_black_path:
        rocket_black = pv.read(rocket_black_path)
        rocket_black.points *= scale
        rocket_black.points -= com
        original_black_points = rocket_black.points.copy()

    # Load airbrake leaves if provided
    airbrake_leaves = []
    original_airbrake_points = []
    if airbrake_leaf_paths:
        for leaf_path in airbrake_leaf_paths:
            leaf = pv.read(leaf_path)
            leaf.points *= scale
            leaf.points -= com
            airbrake_leaves.append(leaf)
            original_airbrake_points.append(leaf.points.copy())

        # Default airbrake positions
        if airbrake_positions is None:
            # These are the radial directions each leaf moves
            airbrake_positions = [
                np.array([1, 0, 0]),
                np.array([0, 1, 0]),
                np.array([-1, 0, 0]),
                np.array([0, -1, 0])
            ]
        else:
            # Normalize provided positions to use as directions
            airbrake_positions = [np.array(pos) / np.linalg.norm(pos) for pos in airbrake_positions]

    # Load data from CSV or simulation
    if csv_path is not None:
        # Load from CSV file
        import pandas as pd
        print(f"Loading flight data from CSV: {csv_path}")
        df = pd.read_csv(csv_path)

        time = np.array(df[csv_time_col])
        z_pos = np.array(df[csv_altitude_col])
        deployment_time = time
        deployment_values = np.array(df[csv_deployment_col]) * airbrake_max_extension

        # No x/y position data - keep rocket at origin
        x_pos = np.zeros_like(time)
        y_pos = np.zeros_like(time)

        # Get velocity data (only vertical component available)
        vx = np.zeros_like(time)
        vy = np.zeros_like(time)
        vz = np.array(df[csv_velocity_col])

        # No rotation data - keep rocket vertical
        e0 = np.ones_like(time)  # w component
        e1 = np.zeros_like(time)  # x component
        e2 = np.zeros_like(time)  # y component
        e3 = np.zeros_like(time)  # z component

        print(f"Loaded {len(time)} data points from CSV")
        print(f"  Time range: {time[0]:.2f} to {time[-1]:.2f} s")
        print(f"  Altitude range: {z_pos.min():.2f} to {z_pos.max():.2f} m")
        print(f"  Velocity range: {vz.min():.2f} to {vz.max():.2f} m/s")
        print(f"  Deployment range: {deployment_values.min():.4f} to {deployment_values.max():.4f} m")

    else:
        # Extract from simulation data
        time = flight.x.source[:, 0]
        x_pos = flight.x.source[:, 1]
        y_pos = flight.y.source[:, 1]
        z_pos = flight.z.source[:, 1] - flight.z.source[0, 1]

        # Get velocity data
        vx = flight.vx.source[:, 1]
        vy = flight.vy.source[:, 1]
        vz = flight.vz.source[:, 1]

        # Get deployment data from controller
        deployment_time = np.array(controller.data['time'])
        deployment_values = np.array(controller.data['deployment']) * airbrake_max_extension

        # Get rotation data
        e0 = flight.e0.source[:, 1]
        e1 = flight.e1.source[:, 1]
        e2 = flight.e2.source[:, 1]
        e3 = flight.e3.source[:, 1]

    if duration is None:
        duration = time[-1] - time[0]

    n_frames = int(duration * fps)
    anim_times = np.linspace(time[0], min(time[0] + duration, time[-1]), n_frames)

    # Interpolate all flight data
    interp_x = np.interp(anim_times, time, x_pos)
    interp_y = np.interp(anim_times, time, y_pos)
    interp_z = np.interp(anim_times, time, z_pos)
    interp_vx = np.interp(anim_times, time, vx)
    interp_vy = np.interp(anim_times, time, vy)
    interp_vz = np.interp(anim_times, time, vz)
    interp_velocity = np.sqrt(interp_vx ** 2 + interp_vy ** 2 + interp_vz ** 2)
    interp_e0 = np.interp(anim_times, time, e0)
    interp_e1 = np.interp(anim_times, time, e1)
    interp_e2 = np.interp(anim_times, time, e2)
    interp_e3 = np.interp(anim_times, time, e3)

    # Interpolate deployment data
    interp_deployment = np.interp(anim_times, deployment_time, deployment_values)

    # Download imagery and get actual coverage
    ground_texture = None
    actual_ground_size = ground_size

    if launch_site_lat is not None:
        result = download_satellite_imagery(
            launch_site_lat, launch_site_lon, ground_size, zoom=imagery_zoom
        )
        if result[0] is not None:
            ground_texture, actual_ground_size = result
        else:
            actual_ground_size = ground_size

    # Create ground plane with correct scale
    ground = pv.Plane(
        center=(0, 0, -com[2] - 0.5),
        direction=(0, 0, 1),
        i_size=result[1],
        j_size=result[1]
    )
    print(f"Ground plane size: {actual_ground_size:.1f}m x {actual_ground_size:.1f}m")

    def transform_mesh_points(original_points, i):

        R = quaternion_to_rotation_matrix(
            interp_e0[i], interp_e1[i], interp_e2[i], interp_e3[i]
        )
        pts = (R @ original_points.T).T
        pts += np.array([interp_x[i], interp_y[i], interp_z[i]])
        return pts, R

    # Video rendering
    plotter = pv.Plotter(off_screen=True)

    import imageio
    # Create output directory if it doesn't exist
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(output_path, fps=fps, codec='libx264')

    percent_done = 10

    for i in range(n_frames):
        plotter.clear()

        # Add flight data text overlay
        text_content = (
            f"Time: {anim_times[i]:.2f} s\n"
            f"Altitude: {interp_z[i]:.2f} m\n"
            f"Speed: {interp_velocity[i]:.2f} m/s\n"
            f"Deployment: {interp_deployment[i] / airbrake_max_extension:.2f}"
        )
        plotter.add_text(text_content, position='upper_left', font_size=12, color='black')

        # Add ground
        if ground_texture:
            plotter.add_mesh(ground, texture=ground_texture)
        else:
            plotter.add_mesh(ground, color='lightgreen', opacity=0.3)

        # Transform and add rocket body (red)
        body_mesh = rocket_body.copy()
        body_mesh.points, R = transform_mesh_points(original_body_points, i)
        plotter.add_mesh(
            body_mesh,
            color="red",
            specular=0.3,
            specular_power = 20,
            smooth_shading=True
        )

        # Transform and add black components
        if rocket_black is not None:
            black_mesh = rocket_black.copy()
            black_mesh.points, _ = transform_mesh_points(original_black_points, i)
            plotter.add_mesh(
                black_mesh,
                color="black",
                ambient=0.2,
                diffuse=0.8,
                specular=0.3,
                specular_power=20,
                smooth_shading=True,
            )

        # Calculate and add airbrakes with deployment animation
        if airbrake_leaves:
            # Use interpolated deployment from simulation
            deployment = interp_deployment[i]

            for leaf_idx, (leaf, orig_points, direction) in enumerate(
                    zip(airbrake_leaves, original_airbrake_points, airbrake_positions)
            ):
                # First rotate the airbrake with the rocket
                leaf_mesh = leaf.copy()
                rotated_points = (R @ orig_points.T).T

                # Then translate outward based on deployment
                # The direction is rotated with the rocket
                world_direction = R @ direction
                rotated_points += world_direction * deployment

                # Finally translate to rocket position
                rotated_points += np.array([interp_x[i], interp_y[i], interp_z[i]])

                leaf_mesh.points = rotated_points
                plotter.add_mesh(
                    leaf_mesh,
                    color="red",
                    smooth_shading=True,
                    specular=0.5,
                    specular_power=30,
                )

        # Add flight trail
        if i > 1:
            trail_points = np.column_stack([interp_x[:i + 1], interp_y[:i + 1], interp_z[:i + 1]])
            trail = pv.Spline(trail_points, n_points=min(len(trail_points) * 2, 1000))
            plotter.add_mesh(trail, color='blue', line_width=3)

        # Camera positioning
        cam_i = min(i + 1, n_frames - 1)
        if camera_option == 0:
            cam_dist = 1.5
            cam_height = max(interp_z[cam_i] + cam_dist / 1000,
                             1) + rocket_z_middle  # Keep camera at least 1m above ground
            focal_z = interp_z[cam_i]  # Focus on middle of rocket
            plotter.camera_position = [
                (interp_x[cam_i] - cam_dist + com[0],
                 interp_y[cam_i] - cam_dist + com[1],
                 cam_height),
                (interp_x[cam_i], interp_y[cam_i], focal_z),
                (0, 0, 1)
            ]
        elif camera_option == 1:
            cam_dist = 1.5
            cam_height = max(interp_z[cam_i] + cam_dist / 2 * (1 + abs(interp_vz[cam_i]) / 50),
                             1) + rocket_z_middle  # Keep camera at least 1m above ground
            focal_z = interp_z[cam_i]  # Focus on middle of rocket
            plotter.camera_position = [
                (interp_x[cam_i] - cam_dist * (1 + interp_vz[cam_i] / 50) + com[0],
                 interp_y[cam_i] - cam_dist * (1 + interp_vz[cam_i] / 50) + com[1],
                 cam_height),
                (interp_x[cam_i], interp_y[cam_i], focal_z),
                (0, 0, 1)
            ]
        else:
            focal_z = interp_z[cam_i]  # Focus on middle of rocket, not COM
            plotter.camera_position = [
                (-100,
                 100,
                 2),
                (interp_x[cam_i], interp_y[cam_i], focal_z),
                (0, 0, 1)
            ]

        plotter.camera.clipping_range = (0.1, result[1] * 3)

        if divmod((i + 1) * 100, (n_frames + 1))[0] >= percent_done:
            print(f"Rendering: {percent_done}% ({i+1} of {n_frames + 1})")
            percent_done += 10

        writer.append_data(plotter.screenshot(return_img=True))

    writer.close()
    plotter.close()

    print(f'Animation saved to {output_path}')