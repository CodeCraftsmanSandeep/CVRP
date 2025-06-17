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
    pat = re.compile(r"Route #\d+:\s*(.*)")
    with open(filepath) as f:
        for line in f:
            m = pat.match(line.strip())
            if m:
                nums = [int(x) for x in m.group(1).split() if x.isdigit()]
                routes.append(nums)
    return routes


def plot_static(coords, routes, depot_id, output_path):
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

    plt.title('Routes')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend(fontsize=6)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.axis('equal')
    # plt.tight_layout(pad=2)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved static plot: {output_path}")


def plot_interactive_html(coords, routes, depot_id, input_name):
    # Generate HTML string with multi-select controls
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

    plot_html = fig.to_html(include_plotlyjs='cdn', full_html=False)
    route_options = '\n'.join([f"<option value='{i}' selected>Route {i}</option>" for i in range(1, len(routes)+1)])
    html = f"""
<html>
<head>
  <title>Interactive Routes - {input_name}</title>
  <script src='https://cdn.plot.ly/plotly-latest.min.js'></script>
</head>
<body>
  <h2>CVRP Routes: {input_name}</h2>
  <div>
    <label for='route-select'>Select routes to display:</label><br>
    <select id='route-select' multiple size='{min(len(routes), 10)}'>
      {route_options}
    </select>
  </div>
  <button id='apply-btn'>Apply Selection</button>
  <div id='plot'>
    {plot_html}
  </div>
  <script>
    const traces = document.querySelectorAll('.js-plotly-plot')[0].data;
    document.getElementById('apply-btn').onclick = function() {{
      const select = document.getElementById('route-select');
      const chosen = Array.from(select.selectedOptions).map(opt => parseInt(opt.value));
      const update = {{visible: []}};
      traces.forEach(trace => {{
        if (trace.name === 'Depot') {{ update.visible.push(true); }}
        else {{ const num = parseInt(trace.name.split(' ')[1]); update.visible.push(chosen.includes(num)); }}
      }});
      Plotly.update('plot', update);
    }};
  </script>
</body>
</html>
"""
    return html


def main():
    parser = argparse.ArgumentParser(description="Plot VRP routes either statically or interactively.")
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

            if args.interactive:
                out_path = os.path.join(args.output, base + '_routes.html')
            else:
                out_path = os.path.join(args.output, base + '_routes.png')

            print(f"Processing {vrp_path}...")
            if not os.path.exists(sol_path):
                print(f"ERROR: Missing solution file {sol_path}")
                continue

            coords, depot_id = parse_vrp(vrp_path)
            routes = parse_sol(sol_path)
            if args.interactive:
                html = plot_interactive_html(coords, routes, depot_id, file)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                with open(out_path, 'w') as f:
                    f.write(html)
                print(f"Saved interactive HTML: {out_path}")
            else:
                plot_static(coords, routes, depot_id, out_path)

    print("All done.")

if __name__ == '__main__':
    main()
