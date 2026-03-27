import argparse
import open3d as o3d

from marchingcubes import (
    create_grid,
    build_mesh_from_sdf,
    make_open3d_mesh,
)
from dualcontouring import (
    normal_from_function,
    dual_contour_3d,
    make_open3d_mesh as make_open3d_mesh_dual,
)
from SDFs import sphere_sdf, cylinder_sdf


def color_mesh(mesh, color):
    mesh.paint_uniform_color(color)
    return mesh


def offset_mesh(mesh, offset):
    mesh.translate(offset, relative=False)
    return mesh


def run_marching(args):
    X, Y, Z = create_grid(args.size, args.bounds)
    if args.shape == "sphere":
        sdf_values = sphere_sdf(X, Y, Z, radius=args.radius)
    elif args.shape == "cylinder":
        sdf_values = cylinder_sdf(X, Y, Z, radius=args.radius, height=args.height)
    verts, faces, normals, values = build_mesh_from_sdf(
        sdf_values,
        level=args.level,
        step_size=args.step_size,
        bounds=args.bounds,
        size=args.size,
    )
    mesh = make_open3d_mesh(verts, faces)
    mesh.compute_vertex_normals()
    print(f"Marching Cubes: {len(verts)} verts, {len(faces)} faces")
    return mesh


def run_dual(args):
    if args.shape == "sphere":
        f = lambda x, y, z: sphere_sdf(x, y, z, args.radius)
        f_normal = normal_from_function(lambda x, y, z: sphere_sdf(x, y, z, args.radius))
    elif args.shape == "cylinder":
        f = lambda x, y, z: cylinder_sdf(x, y, z, args.radius, args.height)
        f_normal = normal_from_function(lambda x, y, z: cylinder_sdf(x, y, z, args.radius, args.height))
    else:
        raise ValueError("Unknown shape")
    verts, faces = dual_contour_3d(
        f,
        f_normal,
        xmin=-args.extent,
        xmax=args.extent,
        ymin=-args.extent,
        ymax=args.extent,
        zmin=-args.extent,
        zmax=args.extent,
    )
    mesh = make_open3d_mesh_dual(verts, faces)
    mesh.compute_vertex_normals()
    print(f"Dual Contour (QEF solver): {len(verts)} verts, {len(faces)} faces")
    return mesh


def main():
    parser = argparse.ArgumentParser(description="Generate and compare marching-cubes + dual-contour meshes")
    parser.add_argument("--shape", choices=["sphere", "cylinder"], default="sphere", help="Implicit shape")
    parser.add_argument("--radius", type=float, default=10.0, help="Sphere radius")
    parser.add_argument("--height", type=float, default=20.0, help="Cylinder height (ignored for sphere)")
    parser.add_argument("--size", type=int, default=25, help="Sample grid resolution for marching cubes")
    parser.add_argument("--bounds", type=float, default=15.0, help="Grid extent ([-bounds, bounds]) for marching cubes")
    parser.add_argument("--extent", type=int, default=12, help="Half-extent for dual contour volume")
    parser.add_argument("--level", type=float, default=0.0, help="Isosurface level for marching cubes")
    parser.add_argument("--step-size", type=int, default=1, help="Marching cubes step size")
    parser.add_argument("--method", choices=["marching", "dual", "both"], default="both", help="Method to run")
    args = parser.parse_args()

    meshes = []

    if args.method in ("marching", "both"):
        m1 = run_marching(args)
        color_mesh(m1, (0.2, 0.7, 0.9))  # Blue
        offset_mesh(m1, (-args.radius * 1.5, 0, 0))
        meshes.append(m1)


    if args.method in ("dual", "both"):
        m2 = run_dual(args)
        color_mesh(m2, (0.9, 0.5, 0.2))  # Orange
        offset_mesh(m2, (args.radius * 1.5, 0, 0))
        meshes.append(m2)
      

    print(f"\nGenerated {len(meshes)} mesh(es): {args.method}")
    print("Close the window to exit")


    if meshes:
        o3d.visualization.draw(meshes, show_ui=True)


if __name__ == "__main__":
    main()