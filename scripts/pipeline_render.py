import os
import glob
import argparse
import numpy as np
import open3d as o3d


def get_point_cloud_files(directory):
    """Scan the target directory for .pcd files and sort them naturally."""
    pcd_files = glob.glob(os.path.join(directory, "*.pcd"))
    pcd_files.sort()
    return pcd_files


def get_background_color(name):
    """Map color names to RGB float vectors."""
    colors = {
        "charcoal": [0.05, 0.05, 0.05],
        "black": [0.0, 0.0, 0.0],
        "white": [1.0, 1.0, 1.0],
        "grey": [0.3, 0.3, 0.3],
        "light_grey": [0.7, 0.7, 0.7],
    }
    return np.asarray(colors.get(name.lower(), [0.05, 0.05, 0.05]))

def open_interactive_viewer(pcd_path, window_width, window_height, point_size, bg_color_name, show_normals=False, no_light=False):
    """
    Opens an interactive Open3D window to view/inspect a single point cloud.
    """
    print("\n" + "=" * 65)
    print("👀 INTERACTIVE POINT CLOUD VIEWER")
    print("=" * 65)
    print(f"File: {os.path.basename(pcd_path)}")
    print(f"Path: {pcd_path}")
    print("👉 Mouse & Keyboard Controls:")
    print("   • Left Click + Drag        : Orbit / Rotate")
    print("   • Ctrl + Left Click + Drag : Pan Camera (Move Viewport)")
    print("   • Shift + Left Click + Drag: Roll / Rotate in Screen Plane")
    print("   • Mouse Wheel              : Zoom In / Out")
    print("   • Press 'N'                : Toggle Normal Vectors (if available)")
    print("   • Press 'L'                : Toggle Lighting")
    print("   • Press 'Q' or Close       : Exit Viewer")
    print("=" * 65 + "\n")

    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        print(f"❌ Error: Failed to load point cloud from '{pcd_path}'")
        return

    num_points = len(pcd.points)
    has_normals = pcd.has_normals()
    has_colors = pcd.has_colors()
    print(f"📊 Cloud Info: {num_points:,} points | Normals: {has_normals} | Colors: {has_colors}")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"RVPoint Viewer - {os.path.basename(pcd_path)}", 
        width=window_width, 
        height=window_height
    )

    opt = vis.get_render_option()
    opt.point_size = point_size
    opt.background_color = get_background_color(bg_color_name)
    opt.light_on = not no_light
    if show_normals and has_normals:
        opt.point_show_normal = True

    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    print("👋 Interactive viewer closed.\n")


def capture_camera_view(sample_pcd_path, window_width, window_height, point_size, bg_color_name, show_normals=False, no_light=False):
    """
    Opens an interactive Open3D window for the user to set the camera angle.
    Returns the pinhole camera parameters upon closing.
    """
    print("\n" + "=" * 65)
    print("📸 INTERACTIVE CAMERA SETUP")
    print("=" * 65)
    print(f"Loading reference cloud: {os.path.basename(sample_pcd_path)}")
    print("👉 Mouse Controls:")
    print("   • Left Click + Drag        : Orbit / Rotate")
    print("   • Ctrl + Left Click + Drag : PAN CAMERA (Move Viewport)")
    print("   • Shift + Left Click + Drag: Roll / Rotate in Screen Plane")
    print("   • Mouse Wheel              : Zoom In / Out")
    print("   • Press 'Q' or Close       : Lock Viewpoint and Start Render")
    print("=" * 65 + "\n")

    pcd = o3d.io.read_point_cloud(sample_pcd_path)
    if pcd.is_empty():
        raise RuntimeError(f"Failed to load point cloud from: {sample_pcd_path}")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Set Camera Viewpoint (Hold Ctrl + Left Drag to Pan)", 
        width=window_width, 
        height=window_height
    )

    # Configure rendering options
    opt = vis.get_render_option()
    opt.point_size = point_size
    opt.background_color = get_background_color(bg_color_name)
    opt.light_on = not no_light
    if show_normals and pcd.has_normals():
        opt.point_show_normal = True

    vis.add_geometry(pcd)
    vis.run()  # Adjust view using Ctrl + Left Drag to pan, then press 'Q'

    view_control = vis.get_view_control()
    cam_params = view_control.convert_to_pinhole_camera_parameters()
    vis.destroy_window()

    print("✅ Camera view angle locked successfully!\n")
    return cam_params


def render_file_offscreen(pcd_path, output_png, cam_params, args):
    """
    Renders a single point cloud stage offscreen with custom settings.
    """
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        print(f"⚠️  Skipping empty file: {pcd_path}")
        return

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Offscreen Render", 
        width=args.width, 
        height=args.height, 
        visible=False
    )

    # Configure rendering options
    opt = vis.get_render_option()
    opt.point_size = args.point_size
    opt.background_color = get_background_color(args.bg_color)
    opt.light_on = not args.no_light

    # Normal Vectors display option
    if args.show_normals and pcd.has_normals():
        opt.point_show_normal = True

    vis.add_geometry(pcd)

    # Apply captured camera parameters if provided
    if cam_params is not None:
        view_control = vis.get_view_control()
        view_control.convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)

    # Force render update before capturing screen image
    vis.poll_events()
    vis.update_renderer()
    
    vis.capture_screen_image(output_png, do_render=True)
    vis.destroy_window()
    print(f"🖼️  Rendered: {os.path.basename(output_png)}")


def main():
    parser = argparse.ArgumentParser(
        description="Interactively view, set camera angle, and render single or batch point cloud files."
    )
    # File & Path Args
    parser.add_argument(
        "positional_input",
        nargs="?",
        type=str,
        default=None,
        help="Path to a single .pcd file or directory containing .pcd files."
    )
    parser.add_argument(
        "-i", "--input", "--input_dir", "--file",
        dest="input_path",
        type=str,
        default=None,
        help="Path to a single .pcd file or directory containing .pcd files."
    )
    parser.add_argument(
        "-o", "--output", "--output_dir",
        dest="output_path",
        type=str,
        default=None,
        help="Path to save PNG render(s) or output directory."
    )
    parser.add_argument(
        "-v", "--view", "--interactive",
        action="store_true",
        help="Open interactive 3D window to view/inspect the point cloud file(s)."
    )
    parser.add_argument(
        "--no_interactive",
        action="store_true",
        help="Skip interactive camera view setup and use default camera for offscreen rendering."
    )

    # Rendering Customization Tweaks
    parser.add_argument(
        "--point_size", 
        type=float, 
        default=1.0, 
        help="Point rendering size in pixels."
    )
    parser.add_argument(
        "--point_style", 
        type=str, 
        choices=["square", "sphere"], 
        default="sphere", 
        help="Shape of rendered points ('square' or 'sphere')."
    )
    parser.add_argument(
        "--bg_color", 
        type=str, 
        choices=["charcoal", "black", "white", "grey", "light_grey"], 
        default="charcoal", 
        help="Background color for rendered images."
    )
    parser.add_argument(
        "--show_normals", 
        action="store_true", 
        help="Force rendering surface normal vectors if available in .pcd."
    )
    parser.add_argument(
        "--no_light", 
        action="store_true", 
        help="Disable scene shading light source."
    )
    parser.add_argument(
        "--width", 
        type=int, 
        default=1280, 
        help="Width of output render image."
    )
    parser.add_argument(
        "--height", 
        type=int, 
        default=720, 
        help="Height of output render image."
    )

    args = parser.parse_args()

    target_path = args.input_path or args.positional_input
    if not target_path:
        parser.print_help()
        print("\n❌ Error: Please specify an input .pcd file or directory.")
        return

    if not os.path.exists(target_path):
        print(f"❌ Error: Path '{target_path}' does not exist.")
        return

    if os.path.isfile(target_path):
        pcd_files = [target_path]
        is_single_file = True
    elif os.path.isdir(target_path):
        pcd_files = get_point_cloud_files(target_path)
        is_single_file = False
        if not pcd_files:
            print(f"❌ Error: No .pcd files found inside '{target_path}'")
            return
    else:
        print(f"❌ Error: Path '{target_path}' is neither a regular file nor a directory.")
        return

    print(f"📁 Target Input: {target_path}")
    if is_single_file:
        print(f"📄 Processing single PCD file: {os.path.basename(target_path)}")
    else:
        print(f"📄 Found {len(pcd_files)} stage file(s):")
        for f in pcd_files:
            print(f"   • {os.path.basename(f)}")

    # Interactive viewing mode (--view / --interactive / -v)
    if args.view:
        for f in pcd_files:
            open_interactive_viewer(
                pcd_path=f,
                window_width=args.width,
                window_height=args.height,
                point_size=args.point_size,
                bg_color_name=args.bg_color,
                show_normals=args.show_normals,
                no_light=args.no_light
            )
        # If user only wanted to view interactively and didn't specify output path, stop here
        if not args.output_path:
            return

    # Determine output destination for offscreen rendering
    if args.output_path:
        if is_single_file and args.output_path.lower().endswith(".png"):
            output_dir = os.path.dirname(os.path.abspath(args.output_path))
            custom_single_output = args.output_path
        else:
            output_dir = args.output_path
            custom_single_output = None
    else:
        custom_single_output = None
        if is_single_file:
            abs_dir = os.path.dirname(os.path.abspath(target_path))
            parent_name = os.path.basename(abs_dir)
            if parent_name and parent_name != "results":
                output_dir = os.path.join("output", "renders", parent_name)
            else:
                output_dir = os.path.join("output", "renders")
        else:
            folder_name = os.path.basename(os.path.normpath(target_path))
            output_dir = os.path.join("output", "renders", folder_name)

    os.makedirs(output_dir, exist_ok=True)

    # Step 1: Set interactive camera angle (unless --no_interactive is set)
    if not args.no_interactive:
        reference_file = pcd_files[0]
        camera_params = capture_camera_view(
            sample_pcd_path=reference_file, 
            window_width=args.width, 
            window_height=args.height, 
            point_size=args.point_size, 
            bg_color_name=args.bg_color,
            show_normals=args.show_normals,
            no_light=args.no_light
        )
    else:
        camera_params = None

    # Step 2: Render offscreen
    print("🚀 Rendering...")
    for file_path in pcd_files:
        if custom_single_output:
            output_png_path = custom_single_output
        else:
            file_stem = os.path.splitext(os.path.basename(file_path))[0]
            output_png_path = os.path.join(output_dir, f"{file_stem}.png")

        render_file_offscreen(
            pcd_path=file_path,
            output_png=output_png_path,
            cam_params=camera_params,
            args=args
        )

    print(f"\n🎉 Render complete! Output saved to: '{os.path.abspath(output_dir)}/'")


if __name__ == "__main__":
    main()
