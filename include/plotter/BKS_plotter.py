import os
import argparse
import matplotlib.pyplot as plt


def parse_vrp(filepath):
    """
    Parse a VRP file in TSPLIB format.
    Returns:
        coords: dict of node_id -> (x, y)
        demands: dict of node_id -> demand (empty if none)
        depot_id: id of depot node
    """
    coords = {}
    demands = {}
    depot_section = False
    coord_section = False
    demand_section = False
    depot_ids = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line == 'NODE_COORD_SECTION':
                coord_section = True
                demand_section = False
                continue
            if line == 'DEMAND_SECTION':
                demand_section = True
                coord_section = False
                continue
            if line == 'DEPOT_SECTION':
                depot_section = True
                coord_section = False
                demand_section = False
                continue
            if coord_section:
                parts = line.split()
                if len(parts) >= 3:
                    node = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    coords[node] = (x, y)
                continue
            if demand_section:
                parts = line.split()
                if len(parts) >= 2:
                    node = int(parts[0])
                    dem = float(parts[1])
                    demands[node] = dem
                continue
            if depot_section:
                if line == '-1' or line == 'EOF':
                    depot_section = False
                else:
                    try:
                        depot_ids.append(int(line))
                    except ValueError:
                        pass
                continue
    depot_id = depot_ids[0] if depot_ids else None
    return coords, demands, depot_id


def plot_instance(coords, demands, depot_id, demand_flag, output_path):
    """Plot the VRP instance, saving to output_path."""
    # Shift coordinates so depot is origin
    if depot_id is None or depot_id not in coords:
        print(f"Warning: depot id {depot_id} not in coords, plotting raw coords")
        base_x, base_y = 0, 0
    else:
        base_x, base_y = coords[depot_id]

    xs = []
    ys = []
    for nid, (x, y) in coords.items():
        xs.append(x - base_x)
        ys.append(y - base_y)

    plt.figure()
    plt.scatter(xs, ys, s=20)
    # Annotate demands if flag set
    if demand_flag and demands:
        for nid, (x, y) in coords.items():
            dem = demands.get(nid, 0)
            plt.text(x - base_x, y - base_y, str(dem), fontsize=6)
    plt.scatter([0], [0], c='red', s=50, label='Depot')
    plt.legend()
    plt.title(os.path.basename(output_path).replace('.png',''))
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"Saved plot to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot VRP instances from .vrp files.")
    parser.add_argument('--input', type=str, required=True, help='Input directory containing .vrp files')
    parser.add_argument('--output', type=str, required=True, help='Output directory for .png plots')
    parser.add_argument('--demand', action='store_true', help='Annotate demands on plot')
    args = parser.parse_args()

    input_dir = args.input
    output_dir = args.output
    demand_flag = args.demand
    print(f"Input directory: {input_dir}")
    print(f"Output directory: {output_dir}")
    print(f"Demand annotation: {demand_flag}")

    # Walk input directory recursively
    for root, dirs, files in os.walk(input_dir):
        for file in files:
            if file.lower().endswith('.vrp'):
                in_path = os.path.join(root, file)
                print(f"Parsing {in_path}...")
                coords, demands, depot_id = parse_vrp(in_path)
                # Build output path
                rel_path = os.path.relpath(in_path, input_dir)
                out_rel = os.path.splitext(rel_path)[0] + '.png'
                out_path = os.path.join(output_dir, out_rel)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                # Plot
                plot_instance(coords, demands, depot_id, demand_flag, out_path)

    print("All done.")


if __name__ == '__main__':
    main()

