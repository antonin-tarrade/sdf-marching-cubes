import open3d as o3d
import argparse
import os
from pathlib import Path


def visualize_group(mesh_dict, title_suffix, gt_mesh_path, pinhole=False):
    """
    Tiles meshes side-by-side.
    mesh_dict: { label_string: Path_to_mesh } (Assumed to be pre-sorted)
    """
    geometries_with_names = []
    offset = 0.0
    spacing = 80  # Adjust if meshes are huge or tiny

    # 1. Handle Ground Truth first (placed on the far left)
    if gt_mesh_path and gt_mesh_path.exists():
        gt_mesh = o3d.io.read_triangle_mesh(str(gt_mesh_path))
        if not gt_mesh.is_empty():
            gt_mesh.compute_vertex_normals()
            gt_mesh.paint_uniform_color([0.6, 0.6, 0.6])  # Distinct grey for GT
            gt_mesh.translate(-gt_mesh.get_center())
            gt_mesh.translate((0, offset, 0))

            geometries_with_names.append({
                "name": f"GT: {gt_mesh_path.name}",
                "geometry": gt_mesh
            })
            offset += spacing

    # 2. Add the selected comparison meshes (ordered left-to-right)
    for label, path in mesh_dict.items():
        mesh = o3d.io.read_triangle_mesh(str(path))
        if mesh.is_empty():
            continue

        mesh.compute_vertex_normals()
        mesh.translate(-mesh.get_center())
        mesh.translate((0, offset, 0))

        geometries_with_names.append({
            "name": label,
            "geometry": mesh
        })

        print(f"Added: {label:<24} | Verts: {len(mesh.vertices):,} "
              f"| Faces: {len(mesh.triangles):,}")
        offset += spacing

    if not geometries_with_names:
        print("No meshes found to display.")
        return

    fov = 30.0 if pinhole else 60.0

    o3d.visualization.draw(
        geometries_with_names,
        title=f"Comparison: {title_suffix}",
        show_ui=True,
        field_of_view=fov
    )


def _extract_resolution(filename: str) -> str:
    """
    Finds the numeric resolution token inside a filename (e.g., 'res256.obj' -> '256')
    """
    digits = "".join([c for c in filename if c.isdigit()])
    return digits if digits else "0"


def find_groundtruth(root_dir: Path, obj_name: str) -> Path | None:
    """
    Looks for a Groundtruth folder at the root level or inside the object folder,
    and tries to find a matching mesh asset.
    """
    # Strategy 1: Check root_dir/Groundtruth/
    gt_dir = root_dir.parent / "Groundtruth"
    if not gt_dir.exists():
        # Strategy 2: Check inside the obj directory itself
        gt_dir = root_dir / "Groundtruth"

    if gt_dir.exists() and gt_dir.is_dir():
        for f in gt_dir.iterdir():
            if f.suffix in ('.obj', '.ply'):
                return f
    return None


def browse_comparison(root_path, pinhole=False):
    """
    Interactive multi-mode comparative browser.
    Handles layout:
       <root_path>/ (or targeted Object Directory)
       └── <MethodName>/
           └── res256.obj, res512.obj
    """
    root = Path(root_path)

    has_subdirs = any(d.is_dir() and not d.name.startswith('.') for d in root.iterdir())
    
    if not has_subdirs:
        print(f"No method directories found inside {root_path}")
        return

    sample_subdirs = [d for d in root.iterdir() if d.is_dir() and d.name.lower() != "groundtruth"]
    
    is_direct_obj_folder = False
    if sample_subdirs:
        for f in sample_subdirs[0].iterdir():
            if f.suffix in ('.obj', '.ply'):
                is_direct_obj_folder = True
                break

    obj_dirs = [root] if is_direct_obj_folder else sample_subdirs

    if not obj_dirs:
        print(f"Could not map target path to expected object-method layout.")
        return

    while True:
        print("\n" + "═" * 50)
        print("  STEP 1 — Select Object Folder")
        print("═" * 50)
        for i, d in enumerate(obj_dirs):
            print(f"  [{i}]  {d.name}")
        print("  [q]  Quit")

        obj_choice = input("\nSelect Object: ").strip()
        if obj_choice.lower() == 'q':
            break

        try:
            selected_obj_dir = obj_dirs[int(obj_choice)]
        except (ValueError, IndexError):
            print("Invalid index.")
            continue

        # Structure: Map[method][resolution] = Path
        db: dict[str, dict[str, Path]] = {}
        all_resolutions: set[str] = set()

        for method_dir in selected_obj_dir.iterdir():
            if not method_dir.is_dir() or method_dir.name.lower() == "groundtruth":
                continue
            
            method_name = method_dir.name
            for file_path in method_dir.iterdir():
                if file_path.suffix in ('.obj', '.ply') and file_path.is_file():
                    res = _extract_resolution(file_path.stem)
                    db.setdefault(method_name, {})[res] = file_path
                    all_resolutions.add(res)

        if not db:
            print(f"No methods or mesh configurations discovered inside {selected_obj_dir.name}")
            continue

        gt_path = find_groundtruth(selected_obj_dir, selected_obj_dir.name)
        if gt_path:
            print(f"-> Linked Ground Truth target: {gt_path.name}")

        while True:
            print(f"\n{'─'*50}")
            print(f"  STEP 2 — {selected_obj_dir.name} | Choose View Mode")
            print(f"{'─'*50}")
            print("  [0]  Compare all METHODS for a specific Resolution")
            print("  [1]  Compare all RESOLUTIONS for a specific Method")
            print("  [b]  Back to Objects")

            mode_choice = input("\nSelect Mode: ").strip()
            if mode_choice.lower() == 'b':
                break

            # --- MODE 0: FIXED RESOLUTION, COMPARE METHODS ---
            if mode_choice == '0':
                sorted_res = sorted(list(all_resolutions), key=lambda x: int(x) if x.isdigit() else 0)
                while True:
                    print(f"\n{'─'*50}")
                    print(f"  Compare Methods | Select Resolution Targets")
                    print(f"{'─'*50}")
                    for i, res in enumerate(sorted_res):
                        avail_count = sum(1 for m in db if res in db[m])
                        print(f"  [{i}]  Resolution: {res}³ ({avail_count} methods available)")
                    print("  [b]  Back to Mode selection")

                    res_choice = input("\nSelect Resolution: ").strip()
                    if res_choice.lower() == 'b':
                        break
                    try:
                        chosen_res = sorted_res[int(res_choice)]
                        
                        # Gather and sort methods alphabetically for consistency
                        payload = {}
                        for method_name in sorted(db.keys()):
                            if chosen_res in db[method_name]:
                                payload[method_name] = db[method_name][chosen_res]
                        
                        visualize_group(payload, f"{selected_obj_dir.name} @ {chosen_res}³", gt_path, pinhole)
                    except (ValueError, IndexError):
                        print("Invalid index.")

            # --- MODE 1: FIXED METHOD, COMPARE RESOLUTIONS (Ascending Order) ---
            elif mode_choice == '1':
                sorted_methods = sorted(list(db.keys()))
                while True:
                    print(f"\n{'─'*50}")
                    print(f"  Compare Resolutions | Select Method Target")
                    print(f"{'─'*50}")
                    for i, method in enumerate(sorted_methods):
                        print(f"  [{i}]  Method: {method} ({len(db[method])} resolutions available)")
                    print("  [b]  Back to Mode selection")

                    method_choice = input("\nSelect Method: ").strip()
                    if method_choice.lower() == 'b':
                        break
                    try:
                        chosen_method = sorted_methods[int(method_choice)]
                        
                        # Extract and sort resolutions numerically for ascending left-to-right order
                        available_res = db[chosen_method].keys()
                        sorted_res_keys = sorted(available_res, key=lambda x: int(x) if x.isdigit() else 0)
                        
                        # Build the ordered payload dictionary
                        payload = {}
                        for res in sorted_res_keys:
                            path = db[chosen_method][res]
                            payload[f"{chosen_method} @ {res}³"] = path
                        
                        visualize_group(payload, f"{selected_obj_dir.name} - Method: {chosen_method}", gt_path, pinhole)
                    except (ValueError, IndexError):
                        print("Invalid index.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Flexible Hierarchy Mesh Comparison Viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--dir',     type=str, help='Path to root or object folder')
    parser.add_argument('--input',   type=str, help='First OBJ/PLY file')
    parser.add_argument('--input2',  type=str, help='Second OBJ/PLY file for comparison')
    parser.add_argument('--gt',      type=str, help='Ground truth mesh')
    parser.add_argument('--pinhole', action='store_true', help='Use a standard pinhole perspective camera matrix')
    args = parser.parse_args()

    if args.dir:
        if os.path.exists(args.dir):
            browse_comparison(args.dir, pinhole=args.pinhole)
        else:
            print(f"Directory not found: {args.dir}")

    elif args.input:
        mesh1 = o3d.io.read_triangle_mesh(args.input)
        meshes_to_draw = []

        if args.gt:
            gt_mesh = o3d.io.read_triangle_mesh(args.gt)
            gt_mesh.compute_vertex_normals()
            gt_mesh.paint_uniform_color([0.6, 0.6, 0.6])
            meshes_to_draw.append(gt_mesh)

        mesh1.translate((80, 0, 0))
        meshes_to_draw.append(mesh1)

        if args.input2:
            mesh2 = o3d.io.read_triangle_mesh(args.input2)
            mesh2.translate((160, 0, 0))
            meshes_to_draw.append(mesh2)

        fov = 30.0 if args.pinhole else 60.0
        o3d.visualization.draw(meshes_to_draw, show_ui=True, field_of_view=fov)

    else:
        parser.print_help()