"""
plot_neighbors_graph.py
-----------------------
Reads a neighbors_full.txt produced by FileNeighborsFull and renders a
colour-coded directed graph showing all node types and their connections.

The graph uses a LAYERED layout that reflects the e-MDB architecture:

    Drive / RobotPurpose          ← motivational level
           ↓
         Goal                     ← goal level
           ↓
         CNode                    ← context (links goals, pnodes, policies)
           ↓
    Policy      PNode             ← action selection / perception model
           ↓
    Perception  WorldModel  …     ← sensory / world model level

Each node type gets a fixed colour and its own band. Arrows show which
nodes know about which other nodes (the neighbors list).

Usage:
    python plot_neighbors_graph.py -f neighbors_full_0.txt
    python plot_neighbors_graph.py -f neighbors_full_0.txt -o vertical
    python plot_neighbors_graph.py -f neighbors_full_0.txt -l dot

The output PNG is saved next to the input file as <stem>_graph_<n>.png.
"""

import argparse
import os
import sys
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx

# ── Architecture layer order (top → bottom) ───────────────────────────────────
# Nodes in layer 0 appear at the top; higher numbers appear lower.
# This reflects the motivational hierarchy of the e-MDB architecture.
LAYER = {
    "Drive":        0,
    "RobotPurpose": 0,
    "Goal":         1,
    "CNode":        2,
    "Policy":       3,
    "PNode":        3,
    "UtilityModel": 3,
    "WorldModel":   4,
    "Perception":   5,
}
DEFAULT_LAYER = 6   # unknown types go at the bottom

# Human-readable band titles (a band can hold several node types).
LAYER_TITLES = {
    0: "Drives / Purposes",
    1: "Goals",
    2: "C-Nodes",
    3: "Policies / P-Nodes / Utility",
    4: "World Models",
    5: "Perceptions",
    DEFAULT_LAYER: "Other",
}

# ── Colour palette per node type ─────────────────────────────────────────────
NODE_COLORS = {
    "Goal":         "#4daf4a",   # green
    "CNode":        "#ff7f00",   # orange
    "PNode":        "#377eb8",   # blue
    "Policy":       "#e41a1c",   # red
    "Drive":        "#984ea3",   # purple
    "Perception":   "#a65628",   # brown
    "WorldModel":   "#888888",   # grey
    "UtilityModel": "#f781bf",   # pink
    "RobotPurpose": "#e6d800",   # yellow
}
DEFAULT_COLOR = "#cccccc"

# Spacing constants for the layered layout (arbitrary data units).
LAYER_GAP = 3.0    # distance between adjacent layer bands
NODE_GAP = 1.6     # distance between adjacent nodes inside a band

# ── Helpers ───────────────────────────────────────────────────────────────────

def strtobool(val: str) -> bool:
    if val.lower() in ("y", "yes", "t", "true", "on", "1"):
        return True
    if val.lower() in ("n", "no", "f", "false", "off", "0"):
        return False
    raise ValueError(f"invalid truth value {val!r}")


def wrap_label(name: str, width: int = 18) -> str:
    """Wrap a node name at underscores so it fits inside the node circle."""
    parts = name.split("_")
    lines, current = [], ""
    for part in parts:
        candidate = (current + "_" + part).lstrip("_")
        if len(candidate) <= width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = part
    if current:
        lines.append(current)
    return "\n".join(lines)


def output_path(file_path: str) -> str:
    """Derive a non-colliding output PNG path from the input file path."""
    stem = re.sub(r"_\d+$", "", os.path.splitext(file_path)[0])
    i = 0
    while os.path.exists(f"{stem}_graph_{i}.png"):
        i += 1
    return f"{stem}_graph_{i}.png"


# ── Graph building ────────────────────────────────────────────────────────────

def build_graph(file_path: str):
    """
    Parse the TSV and return (DiGraph, node_type_map).

    Each node in G gets a 'subset' attribute set to its architecture layer
    number so the layered layout can place it in the right band.
    """
    G = nx.DiGraph()
    node_type_map: dict[str, str] = {}

    with open(file_path, encoding="utf-8") as fh:
        lines = fh.readlines()

    def add_node(name: str, ntype: str):
        # Keep the first non-empty type seen for a node (a node may appear
        # first as someone's neighbor and later as a primary row).
        if name not in node_type_map or not node_type_map[name]:
            node_type_map[name] = ntype
        if name not in G:
            G.add_node(name, subset=LAYER.get(ntype, DEFAULT_LAYER))

    for line in lines:
        fields = line.rstrip("\n").split("\t")
        if len(fields) < 2:
            continue
        node_type, node_name = fields[0].strip(), fields[1].strip()
        if not node_name or (node_type, node_name) == ("NodeType", "NodeName"):
            continue

        add_node(node_name, node_type)

        if len(fields) >= 4:
            neighbor_name = fields[2].strip()
            neighbor_type = fields[3].strip()
            if neighbor_name:
                add_node(neighbor_name, neighbor_type)
                G.add_edge(node_name, neighbor_name)

    return G, node_type_map


# ── Layout ────────────────────────────────────────────────────────────────────

def layered_pos(G: nx.DiGraph, orient: str) -> dict:
    """
    Deterministic layered layout: each architecture layer is a band; nodes
    inside a band are sorted by (type, name) and spaced evenly, centered.

    orient='horizontal'  →  layers run left-to-right (Drives on the left)
    orient='vertical'    →  layers run top-to-bottom (Drives on top)
    """
    by_layer: dict[int, list] = {}
    for n in G.nodes():
        by_layer.setdefault(G.nodes[n]["subset"], []).append(n)

    pos = {}
    for layer, nodes in by_layer.items():
        nodes.sort()
        for i, n in enumerate(nodes):
            along = (i - (len(nodes) - 1) / 2.0) * NODE_GAP
            across = layer * LAYER_GAP
            if orient == "horizontal":
                pos[n] = (across, -along)     # layers as columns
            else:
                pos[n] = (along, -across)     # layers as rows
    return pos


def dot_pos(G: nx.DiGraph, orient: str) -> dict | None:
    """Graphviz 'dot' layout, or None when graphviz/pygraphviz is missing."""
    try:
        rankdir = "LR" if orient == "horizontal" else "TB"
        return nx.nx_agraph.graphviz_layout(G, prog="dot", args=f"-Grankdir={rankdir}")
    except Exception:
        return None


# ── Drawing ───────────────────────────────────────────────────────────────────

def draw_layer_bands(ax, G, pos, orient):
    """Shade one band per architecture layer, sized from the actual node
    positions (only meaningful for the layered layout)."""
    coords_by_layer: dict[int, list] = {}
    for n, (x, y) in pos.items():
        c = x if orient == "horizontal" else y
        coords_by_layer.setdefault(G.nodes[n]["subset"], []).append(c)

    all_x = [x for x, _ in pos.values()]
    all_y = [y for _, y in pos.values()]
    half = LAYER_GAP * 0.38
    for layer, coords in coords_by_layer.items():
        c = sum(coords) / len(coords)
        title = LAYER_TITLES.get(layer, f"layer {layer}")
        if orient == "horizontal":
            ax.axvspan(c - half, c + half, alpha=0.06, color="grey", zorder=0)
            ax.text(c, max(all_y) + NODE_GAP, title,
                    ha="center", va="bottom", fontsize=9,
                    color="#555555", style="italic")
        else:
            ax.axhspan(c - half, c + half, alpha=0.06, color="grey", zorder=0)
            ax.text(min(all_x) - NODE_GAP, c, title,
                    ha="right", va="center", fontsize=9,
                    color="#555555", style="italic", rotation=90)

    # Reserve room for the band titles so they don't collide with the figure
    # title (text does not influence matplotlib's autoscaling).
    if orient == "horizontal":
        ax.set_ylim(min(all_y) - NODE_GAP, max(all_y) + 2.4 * NODE_GAP)
    else:
        ax.set_xlim(min(all_x) - 2.4 * NODE_GAP, max(all_x) + NODE_GAP)


def draw_graph(G: nx.DiGraph, node_type_map: dict, orient: str,
               layout: str, out_path: str):
    """Layout and render the graph, saving to out_path."""
    if G.number_of_nodes() == 0:
        print("Graph is empty — nothing to draw.")
        return

    # ── Positions ─────────────────────────────────────────────────────────────
    pos = None
    used_dot = False
    if layout in ("auto", "dot"):
        pos = dot_pos(G, orient)
        used_dot = pos is not None
        if pos is None and layout == "dot":
            print("graphviz (pygraphviz) not available — falling back to layered layout.")
    if pos is None:
        pos = layered_pos(G, orient)
    print(f"Layout: {'graphviz dot' if used_dot else 'layered architecture bands'}")

    # ── Figure size: driven by the layout extent, within sane bounds ──────────
    xs = [x for x, _ in pos.values()]
    ys = [y for _, y in pos.values()]
    span_x = max(xs) - min(xs) or 1.0
    span_y = max(ys) - min(ys) or 1.0
    if used_dot:
        scale = 0.02  # dot coordinates are in points
    else:
        scale = 1.1
    fig_w = min(max(10, span_x * scale + 4), 34)
    fig_h = min(max(7, span_y * scale + 3), 22)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # ── Layer bands (layered layout only: dot ranks don't match LAYER) ────────
    if not used_dot:
        draw_layer_bands(ax, G, pos, orient)

    # ── Nodes ─────────────────────────────────────────────────────────────────
    node_list = list(G.nodes())
    node_colors = [NODE_COLORS.get(node_type_map.get(n, ""), DEFAULT_COLOR)
                   for n in node_list]
    labels = {n: wrap_label(n) for n in node_list}

    nx.draw_networkx_nodes(
        G, pos, ax=ax,
        nodelist=node_list,
        node_color=node_colors,
        node_size=2200,
        alpha=0.92,
        linewidths=1.2,
        edgecolors="#333333",
    )
    nx.draw_networkx_labels(
        G, pos, ax=ax,
        labels=labels,
        font_size=6.5,
        font_color="black",
    )

    # ── Edges ─────────────────────────────────────────────────────────────────
    nx.draw_networkx_edges(
        G, pos, ax=ax,
        arrowstyle="-|>",
        arrowsize=14,
        edge_color="#444444",
        width=1.2,
        connectionstyle="arc3,rad=0.08",
        min_source_margin=22,
        min_target_margin=22,
    )

    # ── Legend ────────────────────────────────────────────────────────────────
    present_types = sorted({node_type_map.get(n, "Unknown") for n in node_list})
    legend_handles = [
        mpatches.Patch(
            facecolor=NODE_COLORS.get(t, DEFAULT_COLOR),
            edgecolor="#333333",
            label=t,
        )
        for t in present_types
    ]
    ax.legend(
        handles=legend_handles,
        loc="lower right",
        fontsize=9,
        title="Node type",
        title_fontsize=10,
        framealpha=0.9,
    )

    ax.set_title(
        os.path.basename(out_path).replace(".png", ""),
        fontsize=12, pad=12,
    )
    ax.axis("off")
    ax.margins(0.08)
    plt.savefig(out_path, format="png", bbox_inches="tight", dpi=150)
    plt.close()
    print(f"Graph saved to: {out_path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Visualise a neighbors_full.txt file as a layered directed graph."
    )
    parser.add_argument(
        "-f", "--file",
        required=True,
        help="Path to the neighbors_full_N.txt file.",
    )
    parser.add_argument(
        "-s", "--show_graph",
        default="false",
        help="Open the PNG after saving (true/false). Default: false.",
    )
    parser.add_argument(
        "-o", "--orientation",
        default="horizontal",
        choices=["horizontal", "vertical"],
        help="'horizontal' = layers left-to-right (default). 'vertical' = layers top-to-bottom.",
    )
    parser.add_argument(
        "-l", "--layout",
        default="layered",
        choices=["layered", "dot", "auto"],
        help="'layered' (default) = deterministic architecture bands. "
             "'dot' = graphviz hierarchy (needs pygraphviz). "
             "'auto' = dot when available, else layered.",
    )

    args = parser.parse_args()
    file_path = args.file
    show = strtobool(args.show_graph)

    if not os.path.isfile(file_path):
        print(f"Error: file not found: {file_path}", file=sys.stderr)
        sys.exit(1)

    G, node_type_map = build_graph(file_path)
    print(
        f"Parsed {G.number_of_nodes()} nodes, {G.number_of_edges()} edges "
        f"from {os.path.basename(file_path)}"
    )

    out = output_path(file_path)
    draw_graph(G, node_type_map, args.orientation, args.layout, out)

    if show:
        import webbrowser
        webbrowser.open(out)


if __name__ == "__main__":
    main()
