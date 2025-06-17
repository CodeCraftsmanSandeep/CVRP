# README for Input_plotter.py

1️⃣ **What it does:** This script reads TSPLIB-format VRP (.vrp) files and generates 2D plots of customer nodes and depot locations.

2️⃣ **Features:** It can optionally display demand values next to each customer node for better visualization.

3️⃣ **Usage:** Run with `--input <input_dir> --output <output_dir> [--demand]` where input_dir has .vrp files and output_dir will store the PNG plots.

4️⃣ **Output:** The script saves plots as PNG images, preserving the input directory structure within the output directory.

5️⃣ **Requirements:** Python 3, `matplotlib`, `argparse`, and standard Python libraries (`os`).
