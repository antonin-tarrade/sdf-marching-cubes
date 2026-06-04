import open3d as o3d
import argparse
import os
from pathlib import Path


def visualize_group(mesh_dict, resolution, gt_path, obj_name):
    """
    Tiles meshes side-by-side.
    Labels them in the UI with names: 'GT', 'CudaMC', 'FlexiCubes_chunked', etc.
    """
    geometries_with_names = []
    offset = 0.0
    spacing = 80  # Adjust if meshes are huge or tiny

    # 1. Handle Ground Truth first (if found)
    if gt_path:
        gt_mesh = o3d.io.read_triangle_mesh(str(gt_path))
        if not gt_mesh.is_empty():
            gt_mesh.compute_vertex_normals()

            gt_mesh.translate(-gt_mesh.get_center())
            gt_mesh.translate((0, offset, 0))

            geometries_with_names.append({
                "name": f"GT: {gt_path.name}",
                "geometry": gt_mesh
            })
            offset += spacing

    # 2. Add the extracted methods for this resolution
    for method in sorted(mesh_dict.keys()):
        path = mesh_dict[method]
        mesh = o3d.io.read_triangle_mesh(str(path))
        if mesh.is_empty():
            continue

        mesh.translate(-mesh.get_center())
        mesh.translate((0, offset, 0))

        geometries_with_names.append({
            "name": f"{method} @ {resolution}³",
            "geometry": mesh
        })

        print(f"Added: {method:<24} | Verts: {len(mesh.vertices):,} "
              f"| Faces: {len(mesh.triangles):,}")
        offset += spacing

    if not geometries_with_names:
        print("No meshes found to display.")
        return

    o3d.visualization.draw(
        geometries_with_names,
        title=f"Comparison: {obj_name} @ {resolution}³",
        show_ui=True
    )


def _parse_mesh_path(obj_path: Path):
    """
    Parse a mesh filename into (resolution, method_name).

    Expected pattern: <name>_<RES>_<Method>.obj
    where <Method> may contain underscores (e.g. FlexiCubes_chunked).

    Strategy: find the first all-digit token — that is the resolution.
    Everything after it (joined with '_') is the method name.

    Returns (resolution_str, method_str) or (None, None) if unparseable.
    """
    parts = obj_path.stem.split('_')

    # Find index of first numeric token
    res_idx = next((i for i, p in enumerate(parts) if p.isdigit()), None)

    if res_idx is None:
        return None, None                      # no numeric token found
    if res_idx + 1 >= len(parts):
        return None, None                      # nothing after the resolution

    resolution = parts[res_idx]
    method     = '_'.join(parts[res_idx + 1:])  # handles multi-word names
    return resolution, method


def browse_comparison(root_path):
    """
    Interactive hierarchical browser.

    Expected layout (produced by extractsimple.py --mode compare):

        <root>/
        └── <ObjectName>/           ← pass this as --dir, OR the parent
            ├── gt.obj              ← GT at the root of the object folder
            ├── CudaMC/
            │   ├── obj_128_CudaMC.obj
            │   └── obj_256_CudaMC.obj
            ├── FlexiCubes/
            │   └── obj_256_FlexiCubes.obj
            └── FlexiCubes_chunked/
                └── obj_512_FlexiCubes_chunked.obj
    """
    root = Path(root_path)

    # Detect whether the user passed the object folder directly or its parent
    # Heuristic: if the folder contains subfolders that look like method dirs,
    # treat it as an object folder; otherwise treat it as a parent with
    # multiple object folders.
    def _looks_like_object_dir(d: Path) -> bool:
        return any(
            child.is_dir() and not child.name.startswith('.')
            for child in d.iterdir()
        )

    candidates = sorted([d for d in root.iterdir() if d.is_dir()])

    # If root itself looks like it contains method subfolders (not object dirs),
    # treat it as a single object dir
    if all(
        any(f.suffix in ('.obj', '.ply') for f in c.rglob('*')) and not
        any(cc.is_dir() for cc in c.iterdir() if not cc.name.startswith('.'))
        for c in candidates if c.is_dir()
    ):
        obj_dirs = [root]
    else:
        obj_dirs = candidates

    if not obj_dirs:
        print(f"No object folders found in {root_path}")
        return

    while True:
        print("\n" + "═" * 50)
        print("  STEP 1 — Select Object")
        print("═" * 50)
        for i, d in enumerate(obj_dirs):
            print(f"  [{i}]  {d.name}")
        print("  [q]  Quit")

        obj_choice = input("\nSelect: ").strip()
        if obj_choice.lower() == 'q':
            break

        try:
            selected_obj_dir = obj_dirs[int(obj_choice)]
        except (ValueError, IndexError):
            print("Invalid index.")
            continue

        # ── Scan the object folder ────────────────────────────────────────
        # { resolution_str: { method_str: Path } }
        data: dict[str, dict[str, Path]] = {}
        gt_path: Path | None = None

        # GT = first .obj/.ply directly inside the object folder (not in subdirs)
        for f in selected_obj_dir.iterdir():
            if f.suffix in ('.obj', '.ply') and f.is_file():
                gt_path = f
                print(f"\n  GT found: {f.name}")
                break

        # Method meshes = everything inside subfolders
        for obj_path in selected_obj_dir.rglob('*'):
            if obj_path.suffix not in ('.obj', '.ply'):
                continue
            if gt_path and obj_path.resolve() == gt_path.resolve():
                continue

            res, method = _parse_mesh_path(obj_path)
            if res is None:
                continue

            data.setdefault(res, {})[method] = obj_path

        if not data:
            print(f"  No method meshes found under {selected_obj_dir.name}")
            continue

        # ── Resolution selection ──────────────────────────────────────────
        sorted_res = sorted(data.keys(),
                            key=lambda x: int(x) if x.isdigit() else 0)

        while True:
            print(f"\n{'─'*50}")
            print(f"  STEP 2 — {selected_obj_dir.name}  |  Select Resolution")
            print(f"{'─'*50}")
            for i, res in enumerate(sorted_res):
                methods = sorted(data[res].keys())
                print(f"  [{i}]  {res}³   ({len(methods)} methods: "
                      f"{', '.join(methods)})")
            print("  [b]  Back")

            res_choice = input("\nSelect: ").strip()
            if res_choice.lower() == 'b':
                break

            try:
                res_key = sorted_res[int(res_choice)]
                visualize_group(data[res_key], res_key,
                                gt_path, selected_obj_dir.name)
            except (ValueError, IndexError):
                print("Invalid index.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Hierarchical mesh comparison viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Browse comparison results folder
  python viewer.py --dir comparison_results/cow

  # View two meshes side by side
  python viewer.py --input mesh1.obj --input2 mesh2.obj

  # View two meshes against a GT
  python viewer.py --input mesh1.obj --input2 mesh2.obj --gt gt.obj

  # View a single mesh
  python viewer.py --input mesh.obj
        """
    )
    parser.add_argument('--dir',    type=str, help='Path to object comparison folder')
    parser.add_argument('--input',  type=str, help='First OBJ/PLY file')
    parser.add_argument('--input2', type=str, help='Second OBJ/PLY file for comparison')
    parser.add_argument('--gt',     type=str, help='Ground truth mesh (shown in grey)')
    args = parser.parse_args()

    if args.dir:
        if os.path.exists(args.dir):
            browse_comparison(args.dir)
        else:
            print(f"Directory not found: {args.dir}")

    elif args.input:
        mesh1 = o3d.io.read_triangle_mesh(args.input)
        print(f"Mesh1  verts={len(mesh1.vertices):,}  "
              f"faces={len(mesh1.triangles):,}  "
              f"has_normals={mesh1.has_vertex_normals()}  "
              f"has_colors={mesh1.has_vertex_colors()}")

        meshes_to_draw = []

        if args.gt:
            gt_mesh = o3d.io.read_triangle_mesh(args.gt)
            gt_mesh.compute_vertex_normals()
            gt_mesh.paint_uniform_color([0.6, 0.6, 0.6])
            gt_mesh.translate((0, 0, 0))
            meshes_to_draw.append(gt_mesh)

        mesh1.translate((80, 0, 0))
        meshes_to_draw.append(mesh1)

        if args.input2:
            mesh2 = o3d.io.read_triangle_mesh(args.input2)
            print(f"Mesh2  verts={len(mesh2.vertices):,}  "
                  f"faces={len(mesh2.triangles):,}  "
                  f"has_normals={mesh2.has_vertex_normals()}  "
                  f"has_colors={mesh2.has_vertex_colors()}")
            mesh2.translate((160, 0, 0))
            meshes_to_draw.append(mesh2)

        o3d.visualization.draw(meshes_to_draw, show_ui=True)

    else:
        parser.print_help()