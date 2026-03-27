import numpy as np
from skimage import measure
import open3d as o3d

from SDFs import sphere_sdf,cylinder_sdf


def create_grid(size, bounds):
    coords = np.linspace(-bounds, bounds, size)
    return np.meshgrid(coords, coords, coords, indexing="xy")


def build_mesh_from_sdf(sdf_values, level=0, step_size=1, bounds=None, size=None):
    if bounds is not None and size is not None:
        spacing = (2 * bounds / (size - 1),) * 3
    else:
        spacing = (1.0, 1.0, 1.0)
    return measure.marching_cubes(sdf_values, level=level, step_size=step_size, spacing=spacing)


def make_open3d_mesh(verts, faces):
    mesh = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(verts),
        triangles=o3d.utility.Vector3iVector(faces),
    )
    mesh.compute_vertex_normals()
    return mesh


def export_obj(mesh, path):
    o3d.io.write_triangle_mesh(path, mesh)


