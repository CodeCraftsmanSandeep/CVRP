import os
import argparse
import re
import matplotlib.pyplot as plt
import plotly.graph_objects as go


def parse_vrp(filepath):
    coords = {}
    depot_id = None
    in_coord = in_depot = False
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line == 'NODE_COORD_SECTION': in_coord = True; continue
            if line == 'DEMAND_SECTION': in_coord = False; continue
            if line == 'DEPOT_SECTION': in_depot = True; in_coord = False; continue
            if in_coord:
                parts = line.split()
                if len(parts) >= 3:
                    coords[int(parts[0])] = (float(parts[1]), float(parts[2]))
                continue
            if in_depot:
                if line in ('-1','EOF'): in_depot = False
                else:
                    try: depot_id = int(line)
                    except ValueError: pass
    return coords, depot_id


def parse_sol(filepath):
    routes = []
    cost = None
    route_pat = re.compile(r"Route #\d+:\s*(.*)")
    cost_pat = re.compile(r"Cost\s+(\d+(?:\.\d+)?)")
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            m = route_pat.match(line)
            if m:
                nums = [int(x) for x in m.group(1).split() if x.isdigit()]
                routes.append(nums)
                continue
            c = cost_pat.match(line)
            if c:
                cost = float(c.group(1))
    return routes, cost


def plot_static(coords, routes, depot_id, output_path, cost):
    coords0 = {nid - 1: xy for nid, xy in coords.items()}
    depot0 = depot_id - 1 if depot_id is not None else None

    plt.figure(figsize=(8, 6))
    for i, route in enumerate(routes, start=1):
        seq = ([depot0] + route + [depot0]) if depot0 is not None else route
        xs = [coords0[n][0] for n in seq]
        ys = [coords0[n][1] for n in seq]
        plt.plot(xs, ys, marker='o', markersize=2, linewidth=0.5, label=f'Route {i}')

    if depot0 in coords0:
        dx, dy = coords0[depot0]
        plt.scatter([dx], [dy], c='red', s=100, marker='*', label='Depot')
    else:
        print(f"WARNING: Depot ID {depot_id} not found in coords.")

    plt.title(f'Routes - Cost: {cost}')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend(fontsize=6)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.axis('equal')

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved static plot: {output_path}")


def plot_interactive_html(coords, routes, depot_id, input_name, cost, output_path):
    coords0 = {nid - 1: xy for nid, xy in coords.items()}
    depot0 = depot_id - 1 if depot_id is not None else None
    fig = go.Figure()
    for i, route in enumerate(routes, start=1):
        seq = ([depot0] + route + [depot0]) if depot0 is not None else route
        xs = [coords0[n][0] for n in seq]
        ys = [coords0[n][1] for n in seq]
        fig.add_trace(go.Scatter(x=xs, y=ys, mode='lines+markers', name=f'Route {i}', visible=True))
    if depot0 in coords0:
        x, y = coords0[depot0]
        fig.add_trace(go.Scatter(x=[x], y=[y], mode='markers', marker_symbol='star', marker_color='red', marker_size=12, name='Depot', visible=True))

    # Add cost annotation
    fig.update_layout(
        title=f'Interactive Routes - {input_name} (Cost: {cost})'
    )

    plot_html = fig.to_html(include_plotlyjs='cdn', full_html=False)
    html = f"""
<html>
<head>
  <title>Interactive Routes - {input_name}</title>
  <script src='https://cdn.plot.ly/plotly-latest.min.js'></script>
</head>
<body>
  <h2>CVRP Routes: {input_name} (Cost: {cost})</h2>
  <div id='plot'>
    {plot_html}
  </div>
  <label for='route-select'>Show Routes:</label>
  <select id='route-select' multiple size='{max(3, len(routes))}'>
    {''.join([f"<option value='{i}' selected>Route {i}</option>" for i in range(1, len(routes)+1)])}
  </select>
  <button id='apply-btn'>Apply</button>
  <script>
    const plotDiv = document.querySelector('.js-plotly-plot');
    document.getElementById('apply-btn').onclick = function() {{
      const select = document.getElementById('route-select');
      const chosen = Array.from(select.selectedOptions).map(opt => parseInt(opt.value));
      const update = {{visible: []}};
      const traces = plotDiv.data;
      traces.forEach(trace => {{
        if (trace.name === 'Depot') {{ update.visible.push(true); }}
        else {{ const num = parseInt(trace.name.split(' ')[1]); update.visible.push(chosen.includes(num)); }}
      }});
      Plotly.update(plotDiv, update);
    }};
  </script>
</body>
</html>
"""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(html)
    print(f"Saved interactive HTML: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot VRP routes with cost.")
    parser.add_argument('--input', required=True, help='Directory with .vrp and .sol files')
    parser.add_argument('--output', required=True, help='Output directory for plots')
    parser.add_argument('--interactive', action='store_true', help='Generate interactive HTML plots')
    args = parser.parse_args()

    for root, _, files in os.walk(args.input):
        for file in files:
            if not file.lower().endswith('.vrp'): continue
            vrp_path = os.path.join(root, file)
            sol_path = os.path.splitext(vrp_path)[0] + '.sol'
            rel = os.path.relpath(vrp_path, args.input)
            base = os.path.splitext(rel)[0]

            print(f"Processing {vrp_path}...")
            if not os.path.exists(sol_path):
                print(f"ERROR: Missing solution file {sol_path}")
                continue

            coords, depot_id = parse_vrp(vrp_path)
            routes, cost = parse_sol(sol_path)
            if args.interactive:
                out_path = os.path.join(args.output, base + '_routes.html')
                plot_interactive_html(coords, routes, depot_id, file, cost, out_path)
            else:
                out_path = os.path.join(args.output, base + '_routes.png')
                plot_static(coords, routes, depot_id, out_path, cost)

    print("All done.")

if __name__ == '__main__':
    main()
