import os
import argparse
import matplotlib.pyplot as plt
import plotly.graph_objects as go


def parse_vrp(vrp_path):
    coords = {}
    depot = None
    section = None
    with open(vrp_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if line == 'EOF':
                break
            if line.endswith('_SECTION'):
                section = line.replace('_SECTION', '')
                continue
            if section == 'NODE_COORD':
                parts = line.split()
                if len(parts) >= 3:
                    idx = int(parts[0])
                    coords[idx - 1] = (float(parts[1]), float(parts[2]))
            elif section == 'DEPOT':
                if line != '-1':
                    depot = int(line) - 1
    return coords, depot if depot is not None else 0


def parse_graph(graph_path):
    edges = []
    with open(graph_path) as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith('Node'):
                continue
            u = int(line.split()[1].rstrip(':'))
            for part in line.split(':',1)[1].split(')'):
                if '(' in part:
                    v = int(part.strip().lstrip('(').split(',')[0])
                    edges.append((u, v))
    return edges


def plot_static(coords, depot, edges, title, out_path):
    fig, ax = plt.subplots(figsize=(8, 6))
    xs, ys = zip(*coords.values())
    ax.scatter(xs, ys, c='steelblue', s=60, label='Nodes')
    dx, dy = coords[depot]
    ax.scatter([dx], [dy], c='crimson', marker='s', s=120, label='Depot')
    for u, v in edges:
        x0, y0 = coords[u]
        x1, y1 = coords[v]
        ax.annotate('', xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle='->', lw=1, alpha=0.7))
    ax.set_title(title, fontsize=14)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='upper right')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_interactive(coords, depot, edges, title, out_path):
    fig = go.Figure()
    for idx, (x, y) in coords.items():
        fig.add_trace(go.Scatter(
            x=[x], y=[y], mode='markers+text', text=[idx], textposition='top center',
            name=f'Node {idx}', legendgroup=f'{idx}', marker=dict(size=12), hoverinfo='text',
            hovertext=f'Node {idx} (x={x}, y={y})'
        ))
    dx, dy = coords[depot]
    fig.add_trace(go.Scatter(
        x=[dx], y=[dy], mode='markers+text', text=['Depot'], textposition='bottom center',
        name='Depot', marker=dict(symbol='square', size=14, color='firebrick'), hoverinfo='none'
    ))
    for u, v in edges:
        x0, y0 = coords[u]
        x1, y1 = coords[v]
        fig.add_trace(go.Scatter(
            x=[x0, x1], y=[y0, y1], mode='lines', line=dict(width=2), showlegend=False,
            legendgroup=f'{u}', hoverinfo='none'
        ))
        fig.add_trace(go.Scatter(
            x=[x1], y=[y1], mode='markers', marker=dict(symbol='triangle-up', size=8),
            showlegend=False, legendgroup=f'{u}', hoverinfo='none'
        ))
    fig.update_layout(
        title=title, template='plotly_white',
        xaxis_title='X', yaxis_title='Y', hovermode='closest',
        legend=dict(title='Click to remove Node & its edges', itemclick='toggle')
    )
    fig.write_html(out_path)


def main():
    parser = argparse.ArgumentParser(description='Recursively plot VRP/graph pairs using a custom output name')
    parser.add_argument('directory', help='Root directory to search for .vrp/.graph pairs')
    parser.add_argument('output_name', help='Base name for generated plot files (png/html)')
    args = parser.parse_args()

    # Find first matching pair
    for root, _, files in os.walk(args.directory):
        bases = {os.path.splitext(f)[0] for f in files if f.endswith('.vrp')} & {os.path.splitext(f)[0] for f in files if f.endswith('.graph')}
        if bases:
            base = bases.pop()
            vrp_path = os.path.join(root, f"{base}.vrp")
            graph_path = os.path.join(root, f"{base}.graph")
            coords, depot = parse_vrp(vrp_path)
            edges = parse_graph(graph_path)
            title = f"Directed Instance: {args.output_name}"
            png_out = os.path.join(root, f"{args.output_name}.png")
            html_out = os.path.join(root, f"{args.output_name}.html")
            plot_static(coords, depot, edges, title, png_out)
            plot_interactive(coords, depot, edges, title, html_out)
            print(f"Generated plots '{args.output_name}.png' and '{args.output_name}.html' in {root}")
            return
    print("ERROR: No matching .vrp/.graph pair found.")

if __name__ == '__main__':
    main()

