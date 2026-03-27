"Dual contouring implementation with QEF solver."

import numpy as np
import open3d as o3d


def normalize(v):
    n = np.linalg.norm(v)
    if n == 0:
        return np.array([0.0, 0.0, 0.0], dtype=float)
    return v / n

def normal_from_function(f, d=1e-3):
    def norm(x, y, z):
        g = np.array(
            [
                (f(x + d, y, z) - f(x - d, y, z)) / (2 * d),
                (f(x, y + d, z) - f(x, y - d, z)) / (2 * d),
                (f(x, y, z + d) - f(x, y, z - d)) / (2 * d),
            ],
            dtype=float,
        )
        return normalize(g)

    return norm


def solve_qef(positions, normals):
    """Solve Quadric Error Function to find optimal vertex position.
    
    Minimizes sum of squared distances from point to planes defined by (position, normal) pairs.
    """
    if len(positions) == 0:
        return None
    
    # Build ATA and ATb for least squares: minimize ||Ax - b||^2
    A = np.array(normals, dtype=np.float64)
    b = np.array([np.dot(normals[i], positions[i]) for i in range(len(positions))], dtype=np.float64)
    
    try:
        # Solve normal equations: (A^T A) x = A^T b
        ATA = A.T @ A
        ATb = A.T @ b
        
        # Add small regularization for numerical stability
        ATA += np.eye(3) * 1e-6
        
        x = np.linalg.solve(ATA, ATb)
        return x
    except np.linalg.LinAlgError:
        # If singular, fall back to centroid
        return np.mean(positions, axis=0)


def dual_contour_3d_find_best_vertex(f, f_normal, x, y, z):
    """Find best vertex for cell using QEF """
    
    corners = np.array([
        [x, y, z], [x+1, y, z], [x, y+1, z], [x+1, y+1, z],
        [x, y, z+1], [x+1, y, z+1], [x, y+1, z+1], [x+1, y+1, z+1]
    ], dtype=float)
    
    corner_vals = np.array([f(*c) for c in corners])
      
    # Collect edge intersection points and normals
    positions = []
    normals = []
    
    # Check all 12 edges
    edges = [
        (0, 1), (2, 3), (4, 5), (6, 7),  # edges parallel to x
        (0, 2), (1, 3), (4, 6), (5, 7),  # edges parallel to y
        (0, 4), (1, 5), (2, 6), (3, 7),  # edges parallel to z
    ]
    
    for i, j in edges:
        val_i, val_j = corner_vals[i], corner_vals[j]
        
        # Check if edge crosses surface
        if (val_i > 0) != (val_j > 0):
            # Interpolate position along edge
            t = abs(val_i) / (abs(val_i) + abs(val_j) + 1e-12)
            pos = corners[i] + t * (corners[j] - corners[i])
            
            # Get normal at intersection
            normal = f_normal(pos[0], pos[1], pos[2])
            
            positions.append(pos)
            normals.append(normal)
    
    if len(positions) > 0:
        # Solve QEF for optimal vertex
        best_v = solve_qef(positions, normals)
        if best_v is not None:
            # Clamp to cell bounds
            best_v = np.clip(best_v, [x, y, z], [x+1, y+1, z+1])
            return best_v
    
    # Fallback to cell center
    return np.array([x + 0.5, y + 0.5, z + 0.5], dtype=float)


def add_quad(faces, i0, i1, i2, i3, flip=False):
    if flip:
        faces.append([i0, i1, i2])
        faces.append([i0, i2, i3])
    else:
        faces.append([i0, i2, i1])
        faces.append([i0, i3, i2])


def dual_contour_3d(
    f,
    f_normal,
    xmin=-12,
    xmax=12,
    ymin=-12,
    ymax=12,
    zmin=-12,
    zmax=12,
):
    vert_array = []
    vert_indices = {}

    # First pass: create vertices at optimal positions
    for x in range(xmin, xmax):
        for y in range(ymin, ymax):
            for z in range(zmin, zmax):
                vert = dual_contour_3d_find_best_vertex(f, f_normal, x, y, z)
                if vert is None:
                    continue
                vert_indices[(x, y, z)] = len(vert_array)
                vert_array.append(vert)

    # Second pass: create faces
    faces = []
    for x in range(xmin, xmax):
        for y in range(ymin, ymax):
            for z in range(zmin, zmax):
                if x <= xmin or y <= ymin or z <= zmin:
                    continue

                solid1 = f(x, y, z) > 0
                solid2 = f(x, y, z + 1) > 0
                if solid1 != solid2:
                    keys = [(x - 1, y - 1, z), (x, y - 1, z), (x, y, z), (x - 1, y, z)]
                    ids = [vert_indices.get(k) for k in keys]
                    if None not in ids:
                        add_quad(faces, *ids, flip=solid2)

                solid1 = f(x, y, z) > 0
                solid2 = f(x, y + 1, z) > 0
                if solid1 != solid2:
                    keys = [(x - 1, y, z - 1), (x, y, z - 1), (x, y, z), (x - 1, y, z)]
                    ids = [vert_indices.get(k) for k in keys]
                    if None not in ids:
                        add_quad(faces, *ids, flip=solid1)

                solid1 = f(x, y, z) > 0
                solid2 = f(x + 1, y, z) > 0
                if solid1 != solid2:
                    keys = [(x, y - 1, z - 1), (x, y, z - 1), (x, y, z), (x, y - 1, z)]
                    ids = [vert_indices.get(k) for k in keys]
                    if None not in ids:
                        add_quad(faces, *ids, flip=solid2)

    return np.array(vert_array, dtype=np.float64), np.array(faces, dtype=np.int32)


def make_open3d_mesh(verts, faces):
    mesh = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(verts),
        triangles=o3d.utility.Vector3iVector(faces),
    )
    mesh.compute_vertex_normals()
    return mesh


def export_obj(mesh, path):
    o3d.io.write_triangle_mesh(path, mesh)