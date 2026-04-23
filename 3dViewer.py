import open3d as o3d
import argparse


def visualize_mesh(mesh):
    """
    Visualizes a mesh using Open3D.
    
    Args:
        mesh (o3d.geometry.TriangleMesh): The mesh to visualize.
    """
    if not isinstance(mesh, o3d.geometry.TriangleMesh):
        raise ValueError("Input must be an Open3D TriangleMesh.")
    
    # Create a visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Mesh Visualization', width=800, height=600)
    
    # Add the mesh to the visualizer
    vis.add_geometry(mesh)
    
    # Set up the view
    vis.get_render_option().background_color = [0.1, 0.1, 0.1]  # Dark background
    vis.get_render_option().mesh_show_back_face = True  # Show back faces
    
    # Run the visualizer
    vis.run()
    
    # Destroy the window after visualization
    vis.destroy_window()


def compare_meshes(mesh1, mesh2):
    """
    Compares two meshes and visualizes them side by side for comparison.
    
    Args:
        mesh1 (o3d.geometry.TriangleMesh): The first mesh to compare.
        mesh2 (o3d.geometry.TriangleMesh): The second mesh to compare.
    """
    if not isinstance(mesh1, o3d.geometry.TriangleMesh) or not isinstance(mesh2, o3d.geometry.TriangleMesh):
        raise ValueError("Both inputs must be Open3D TriangleMesh objects.")
    
    mesh1.translate((0, -mesh2.get_center()[1] - 50, 0))  # Move mesh1 to the left
    mesh2.translate((0, -mesh2.get_center()[1] + 50, 0))  # Move mesh2 to the right
    meshes = [mesh1, mesh2]
    #Add nb of vertices and faces to the printout
    print(f"Mesh 1: {len(mesh1.vertices)} vertices, {len(mesh1.triangles)} faces")
    print(f"Mesh 2: {len(mesh2.vertices)} vertices, {len(mesh2.triangles)} faces")
    o3d.visualization.draw(meshes, show_ui=True)


if __name__ == "__main__":
    # one or two mesh generically
    argument_parser = argparse.ArgumentParser(description='Visualize a mesh from an OBJ file')
    argument_parser.add_argument('--input', type=str, required=True, help='Path to the OBJ file to visualize')
    argument_parser.add_argument('--input2', type=str, help='Path to the second OBJ file for comparison')
    args = argument_parser.parse_args()
    # Load the first mesh
    mesh1 = o3d.io.read_triangle_mesh(args.input)
    if mesh1.is_empty():
        print(f"Error: Could not load mesh from {args.input}")
        exit(1)
    # Visualize the first mesh    
    if args.input2 is None:
        visualize_mesh(mesh1)
    else:
        # Load the second mesh
        mesh2 = o3d.io.read_triangle_mesh(args.input2)
        if mesh2.is_empty():
            print(f"Error: Could not load mesh from {args.input2}")
            exit(1)
        # Compare the two meshes
        compare_meshes(mesh1, mesh2)
