"""
plot_neighbors_graph.py
-----------------------
Reads a neighbors_full.txt produced by FileNeighborsFull and generates a
SELF-CONTAINED, INTERACTIVE HTML view of the e-MDB cognitive graph.

Why HTML instead of a static image: a real run has hundreds of nodes
(PNodes, CNodes, Goals, Policies, ...), which is unreadable as a single PNG.
The HTML lets you:
  * show/hide whole node types / architecture layers (e.g. hide Perceptions),
  * hide, isolate or delete individual nodes (right-click a node),
  * drag nodes to arrange them, then SAVE that arrangement to a JSON config and
    LOAD it back on the same or a similar file (positions + hidden/deleted state),
  * search for a node (Enter / Shift+Enter to cycle matches),
  * export a PNG, print the full graph to PDF, or export editable draw.io XML,
    choosing the output file name.

The single layout is "layered by type": one horizontal band per node type.

Arrows follow ACTIVATION FLOW (neighbour -> node): a node's "neighbors" are its
inputs, so a CNode is pointed at by its PNode / Goal / WorldModel, and flow runs
Perceptions -> ... -> Drives. (The previous version drew these reversed.)

Usage:
    python plot_neighbors_graph.py -f neighbors_full_0.txt
    python plot_neighbors_graph.py -f neighbors_full_0.txt -o graph.html
    python plot_neighbors_graph.py -f neighbors_full_0.txt --open

Only the Python standard library is required; rendering happens in the browser
via Cytoscape.js (loaded from a CDN, so an internet connection is needed the
first time you open the file).
"""

import argparse
import json
import os
import sys

# ── Architecture layers (0 = top of the motivational hierarchy) ───────────────
LAYER = {
    "Drive": 0, "RobotPurpose": 0,
    "Goal": 1,
    "CNode": 2,
    "Policy": 3, "PNode": 3, "UtilityModel": 3,
    "WorldModel": 4,
    "Perception": 5,
}
DEFAULT_LAYER = 6

LAYER_TITLES = {
    0: "Drives / Purposes", 1: "Goals", 2: "C-Nodes",
    3: "Policies / P-Nodes / Utility", 4: "World Models",
    5: "Perceptions", DEFAULT_LAYER: "Other",
}

NODE_COLORS = {
    "Goal": "#4daf4a", "CNode": "#ff7f00", "PNode": "#377eb8",
    "Policy": "#e41a1c", "Drive": "#984ea3", "Perception": "#a65628",
    "WorldModel": "#888888", "UtilityModel": "#f781bf", "RobotPurpose": "#e6d800",
}
DEFAULT_COLOR = "#cccccc"

# "Layered by type" gives EACH node type its own band (top -> bottom), unlike
# the coarser architecture LAYER grouping above (which is still used only to
# organise the sidebar toggles). Absent types are compacted out.
TYPE_ORDER = ["Drive", "RobotPurpose", "Goal", "CNode", "Policy", "PNode",
              "UtilityModel", "WorldModel", "Perception"]
TYPE_LAYER = {t: i for i, t in enumerate(TYPE_ORDER)}
DEFAULT_TYPE_LAYER = len(TYPE_ORDER)

# Layered-layout spacing (browser pixels).
COL_GAP = 210        # horizontal centre gap (nodes size to their label, max ~175px wide)
LAYER_V_GAP = 150    # vertical gap between type bands
BARY_SWEEPS = 8      # barycentre ordering passes to reduce edge crossings


def parse(file_path):
    """Parse the TSV into (node_type_map, edges). Edges are (source, target)
    with source=neighbour, target=node so arrows follow activation flow."""
    node_type = {}
    edges = []
    seen_edges = set()

    def add_node(name, ntype):
        if name and (name not in node_type or not node_type[name]):
            node_type[name] = ntype

    with open(file_path, encoding="utf-8") as fh:
        for line in fh:
            f = line.rstrip("\n").split("\t")
            if len(f) < 2:
                continue
            ntype, name = f[0].strip(), f[1].strip()
            if not name or (ntype, name) == ("NodeType", "NodeName"):
                continue
            add_node(name, ntype)
            if len(f) >= 4 and f[2].strip():
                nb_name, nb_type = f[2].strip(), f[3].strip()
                add_node(nb_name, nb_type)
                key = (nb_name, name)           # neighbour -> node
                if key not in seen_edges:
                    seen_edges.add(key)
                    edges.append(key)
    return node_type, edges


def layered_positions(node_type, edges):
    """One horizontal band PER NODE TYPE (single row each), with nodes ordered
    inside each band by the median x of their non-hub neighbours. Each type gets
    its own row and the median heuristic keeps the chains between types aligned,
    so edge crossings stay low."""
    by_band = {}
    for name, ntype in node_type.items():
        by_band.setdefault(TYPE_LAYER.get(ntype, DEFAULT_TYPE_LAYER), []).append(name)
    order = {b: sorted(v) for b, v in by_band.items()}

    adj = {}
    for s, t in edges:
        adj.setdefault(s, []).append(t)
        adj.setdefault(t, []).append(s)
    deg = {n: len(v) for n, v in adj.items()}
    # Hubs (a WorldModel / Perceptions wired to almost everything) are ignored
    # when ordering, so nodes line up by their meaningful 1-to-1 chains rather
    # than all being dragged toward the hub. Hub edges stay long — fine, and the
    # hubs can be hidden in the viewer anyway.
    HUB_DEG = 12

    def space(names):
        """Evenly space a layer's nodes, centred on x=0. Returns {name: x}."""
        width = (len(names) - 1) * COL_GAP
        return {n: i * COL_GAP - width / 2 for i, n in enumerate(names)}

    xpos = {}
    for names in order.values():
        xpos.update(space(names))

    layers = sorted(order)
    for sweep in range(BARY_SWEEPS):
        seq = layers if sweep % 2 == 0 else layers[::-1]
        for L in seq:
            names = order[L]
            def bary(n):
                # Median of non-hub neighbours' x (fall back to all neighbours
                # if a node only touches hubs). Median + hub exclusion keeps the
                # real chains aligned and ignores the everything-connected hubs.
                xs = sorted(xpos[m] for m in adj.get(n, [])
                            if m in xpos and deg.get(m, 0) <= HUB_DEG)
                if not xs:
                    xs = sorted(xpos[m] for m in adj.get(n, []) if m in xpos)
                if not xs:
                    return xpos[n]
                k = len(xs)
                return xs[k // 2] if k % 2 else (xs[k // 2 - 1] + xs[k // 2]) / 2
            names.sort(key=bary)
            xpos.update(space(names))   # re-centre after reordering

    band_y = {b: i for i, b in enumerate(layers)}   # compact absent types out
    return {n: {"x": xpos[n], "y": band_y[L] * LAYER_V_GAP}
            for L in layers for n in order[L]}


def build_elements(node_type, edges):
    pos = layered_positions(node_type, edges)
    nodes = []
    for name, ntype in sorted(node_type.items()):
        short = name if len(name) <= 40 else name[:38] + "…"
        nodes.append({
            "data": {
                "id": name, "label": short, "type": ntype or "Unknown",
                "color": NODE_COLORS.get(ntype, DEFAULT_COLOR),
                "layer": LAYER.get(ntype, DEFAULT_LAYER),
            },
            "position": pos.get(name, {"x": 0, "y": 0}),
        })
    edge_els = [{"data": {"id": f"e{i}", "source": s, "target": t}}
                for i, (s, t) in enumerate(edges)]
    return nodes + edge_els


def type_summary(node_type):
    counts = {}
    for ntype in node_type.values():
        counts[ntype] = counts.get(ntype, 0) + 1
    rows = [{"type": t, "color": NODE_COLORS.get(t, DEFAULT_COLOR),
             "count": c, "layer": LAYER.get(t, DEFAULT_LAYER),
             "layerTitle": LAYER_TITLES.get(LAYER.get(t, DEFAULT_LAYER), "Other")}
            for t, c in counts.items()]
    rows.sort(key=lambda r: (r["layer"], r["type"]))
    return rows


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>__TITLE__</title>
<script src="https://unpkg.com/cytoscape@3.30.2/dist/cytoscape.min.js"></script>
<style>
  :root { --bg:#f7f8fa; --panel:#ffffff; --line:#e2e5ea; --ink:#222; --muted:#666; }
  * { box-sizing: border-box; }
  html,body { margin:0; height:100%; font-family: system-ui, Arial, sans-serif; color:var(--ink); }
  #app { display:flex; height:100vh; }
  #panel { width:300px; min-width:300px; background:var(--panel); border-right:1px solid var(--line);
           overflow-y:auto; padding:14px 14px 40px; }
  #cy { flex:1; background:var(--bg); }
  h1 { font-size:15px; margin:0 0 2px; }
  .sub { color:var(--muted); font-size:12px; margin-bottom:12px; }
  .section { border-top:1px solid var(--line); padding:12px 0 4px; }
  .section h2 { font-size:12px; text-transform:uppercase; letter-spacing:.04em; color:var(--muted); margin:0 0 8px; }
  .row { display:flex; align-items:center; gap:8px; font-size:13px; padding:3px 0; }
  .row label { display:flex; align-items:center; gap:8px; cursor:pointer; flex:1; }
  .swatch { width:13px; height:13px; border-radius:3px; border:1px solid #0002; flex:none; }
  .count { color:var(--muted); font-size:11px; }
  .layerlabel { font-size:11px; color:var(--muted); margin:8px 0 2px; font-style:italic; }
  input[type=text], select { width:100%; padding:6px 8px; border:1px solid var(--line); border-radius:6px; font-size:13px; }
  button { width:100%; padding:7px 8px; border:1px solid var(--line); background:#fff; border-radius:6px;
           font-size:13px; cursor:pointer; margin-top:6px; }
  button:hover { background:#f0f2f5; }
  button.primary { background:#2563eb; color:#fff; border-color:#2563eb; }
  button.primary:hover { background:#1d4ed8; }
  .mini { display:flex; gap:6px; }
  .mini button { margin-top:0; }
  .hint { font-size:11px; color:var(--muted); margin-top:8px; line-height:1.4; }
  #menu { position:fixed; display:none; background:#fff; border:1px solid var(--line);
          border-radius:8px; box-shadow:0 6px 24px #0002; z-index:10; overflow:hidden; min-width:150px; }
  #menu div { padding:8px 12px; font-size:13px; cursor:pointer; }
  #menu div:hover { background:#f0f2f5; }
  #tip { position:fixed; display:none; background:#111; color:#fff; padding:4px 8px;
         border-radius:4px; font-size:12px; pointer-events:none; z-index:20;
         max-width:420px; word-break:break-all; }
  @media print {
    #panel, #menu { display:none !important; }
    #cy { position:absolute; inset:0; }
  }
</style>
</head>
<body>
<div id="app">
  <div id="panel">
    <h1>__TITLE__</h1>
    <div class="sub" id="stats"></div>

    <div class="section">
      <h2>Layers &amp; node types</h2>
      <div class="mini">
        <button id="allOn">All</button>
        <button id="allOff">None</button>
      </div>
      <div id="types"></div>
    </div>

    <div class="section">
      <h2>Arrangement config</h2>
      <button id="saveCfg">Save config (.json)</button>
      <button id="loadCfg">Load config…</button>
      <input id="cfgFile" type="file" accept=".json,application/json" style="display:none"/>
      <div class="hint">Saves your arrangement — node positions (drag them to
        rearrange), which types/nodes are hidden, and deletions. Load it into the
        same or a similar file to restore the same view.</div>
    </div>

    <div class="section">
      <h2>Find node</h2>
      <input id="search" type="text" placeholder="type part of a node name…"/>
      <div class="hint"><b id="searchCount"></b> Enter = next match,
        Shift+Enter = previous. Right-click any node to hide / isolate / delete it.</div>
    </div>

    <div class="section">
      <h2>Export</h2>
      <input id="fname" type="text" value="__DEFAULT_FILENAME__"/>
      <button class="primary" id="png">Download PNG</button>
      <button id="pdf">Print / Save as PDF</button>
      <button id="drawio">Download draw.io XML (editable)</button>
    </div>

    <div class="section">
      <button id="reset">Reset (restore hidden / deleted)</button>
    </div>
  </div>
  <div id="cy"></div>
</div>

<div id="menu">
  <div data-act="expand">Expand neighbours</div>
  <div data-act="neighbours">Show only this + neighbours</div>
  <div data-act="isolate">Isolate (hide the rest)</div>
  <div data-act="rename">Rename / alias…</div>
  <div data-act="hide">Hide node</div>
  <div data-act="delete">Delete node</div>
</div>
<div id="tip"></div>

<script>
const ELEMENTS = __ELEMENTS_JSON__;
const TYPES = __TYPES_JSON__;
const ORIGINAL = JSON.parse(JSON.stringify(ELEMENTS));

const cy = cytoscape({
  container: document.getElementById('cy'),
  elements: ELEMENTS,
  wheelSensitivity: 0.2,
  style: [
    { selector: 'node', style: {
        'background-color': 'data(color)', 'label': 'data(label)',
        'font-size': 9, 'text-wrap': 'wrap', 'text-max-width': 160,
        'text-valign': 'center', 'text-halign': 'center',
        'width': 'label', 'height': 'label', 'padding': 6,
        'shape': 'round-rectangle', 'border-width': 1, 'border-color': '#33333355',
        'color': '#111' } },
    { selector: 'edge', style: {
        'width': 1.2, 'line-color': '#9aa1ac', 'target-arrow-color': '#9aa1ac',
        'target-arrow-shape': 'triangle', 'arrow-scale': 0.9,
        'curve-style': 'bezier', 'opacity': 0.75 } },
    { selector: '.faded', style: { 'opacity': 0.08 } },
    { selector: 'node.hl', style: { 'border-width': 4, 'border-color': '#2563eb' } },
    { selector: 'node.hl-current', style: { 'border-width': 6, 'border-color': '#ff6d00' } },
  ],
  layout: { name: 'preset', fit: true, padding: 40 },
});

// The default layered-by-type positions, so the arrangement can be reset.
const PRESET_POS = {};
ORIGINAL.forEach(el => { if (el.position) PRESET_POS[el.data.id] = el.position; });
function applyLayered() {
  cy.layout({ name: 'preset', positions: n => PRESET_POS[n.id()] || { x: 0, y: 0 },
              animate: false, fit: true, padding: 40 }).run();
}
// Default display label (full name when short enough, else truncated) — matches
// the Python side and is used to revert when an alias is cleared.
function defLabel(id) { return id.length <= 40 ? id : id.slice(0, 38) + '…'; }

function refreshStats() {
  const n = cy.nodes(':visible').length, e = cy.edges(':visible').length;
  document.getElementById('stats').textContent =
      n + ' nodes · ' + e + ' edges visible';
}
refreshStats();

// ── Type / layer toggles ────────────────────────────────────────────────────
const typesDiv = document.getElementById('types');
let lastLayer = null;
TYPES.forEach(t => {
  if (t.layer !== lastLayer) {
    const l = document.createElement('div');
    l.className = 'layerlabel'; l.textContent = t.layerTitle;
    typesDiv.appendChild(l); lastLayer = t.layer;
  }
  const row = document.createElement('div');
  row.className = 'row';
  row.innerHTML =
    '<label><input type="checkbox" checked data-type="' + t.type + '">' +
    '<span class="swatch" style="background:' + t.color + '"></span>' +
    t.type + '</label><span class="count">' + t.count + '</span>';
  typesDiv.appendChild(row);
});
function applyType(type, on) {
  cy.nodes('[type="' + type + '"]').style('display', on ? 'element' : 'none');
  refreshStats();
}
typesDiv.addEventListener('change', e => {
  if (e.target.matches('input[data-type]'))
    applyType(e.target.getAttribute('data-type'), e.target.checked);
});
document.getElementById('allOn').onclick = () => {
  typesDiv.querySelectorAll('input[data-type]').forEach(c => { c.checked = true; applyType(c.dataset.type, true); });
};
document.getElementById('allOff').onclick = () => {
  typesDiv.querySelectorAll('input[data-type]').forEach(c => { c.checked = false; applyType(c.dataset.type, false); });
};


// ── Search ───────────────────────────────────────────────────────────────────
let searchQuery = null, searchMatches = [], searchIndex = -1;
const searchCount = document.getElementById('searchCount');
document.getElementById('search').addEventListener('keydown', e => {
  if (e.key !== 'Enter') return;
  e.preventDefault();
  const q = e.target.value.trim().toLowerCase();
  if (!q) {
    cy.nodes().removeClass('hl hl-current');
    searchQuery = null; searchMatches = []; searchIndex = -1; searchCount.textContent = '';
    return;
  }
  if (q !== searchQuery) {                       // new query: (re)collect matches
    searchQuery = q;
    searchMatches = cy.nodes().filter(
      n => n.id().toLowerCase().includes(q) && n.style('display') !== 'none').toArray();
    searchIndex = -1;
    cy.nodes().removeClass('hl hl-current');
    searchMatches.forEach(n => n.addClass('hl'));
  }
  if (!searchMatches.length) { searchCount.textContent = 'no matches'; return; }
  const dir = e.shiftKey ? -1 : 1;               // Enter = next, Shift+Enter = prev
  searchIndex = (searchIndex + dir + searchMatches.length) % searchMatches.length;
  cy.nodes().removeClass('hl-current');
  const cur = searchMatches[searchIndex];
  cur.addClass('hl-current');
  cy.animate({ center: { eles: cur }, zoom: 1.2 }, { duration: 250 });
  searchCount.textContent = (searchIndex + 1) + ' / ' + searchMatches.length;
});

// ── Right-click context menu ─────────────────────────────────────────────────
const menu = document.getElementById('menu');
let menuNode = null;
document.getElementById('cy').addEventListener('contextmenu', e => e.preventDefault());
cy.on('cxttap', 'node', evt => {
  menuNode = evt.target;
  const oe = evt.originalEvent || {};
  menu.style.left = (oe.clientX || 0) + 'px';
  menu.style.top = (oe.clientY || 0) + 'px';
  menu.style.display = 'block';
});
cy.on('tap', () => { menu.style.display = 'none'; });

// ── Hover tooltip: labels are truncated, so show the full name on hover ───────
const tip = document.getElementById('tip');
cy.on('mouseover', 'node', evt => {
  tip.textContent = evt.target.id();
  const oe = evt.originalEvent || {};
  tip.style.left = ((oe.clientX || 0) + 12) + 'px';
  tip.style.top = ((oe.clientY || 0) + 12) + 'px';
  tip.style.display = 'block';
});
cy.on('mouseout', 'node', () => { tip.style.display = 'none'; });
menu.addEventListener('click', e => {
  const act = e.target.getAttribute('data-act');
  if (!act || !menuNode) return;
  if (act === 'hide') menuNode.style('display', 'none');
  else if (act === 'delete') menuNode.remove();
  else if (act === 'isolate') { cy.nodes().style('display', 'none'); menuNode.style('display', 'element'); }
  else if (act === 'expand') {
    // Progressive exploration: reveal this node's neighbours (and their edges)
    // while keeping whatever is already on screen.
    menuNode.style('display', 'element');
    menuNode.neighborhood().style('display', 'element');
  }
  else if (act === 'neighbours') {
    const keep = menuNode.closedNeighborhood().nodes();
    cy.nodes().style('display', 'none'); keep.style('display', 'element');
  }
  else if (act === 'rename') {
    const v = prompt('Display name / alias for this node\n(' + menuNode.id() + ')',
                     menuNode.data('label'));
    if (v !== null) {
      const t = v.trim();
      menuNode.data('alias', t);
      menuNode.data('label', t || defLabel(menuNode.id()));
    }
  }
  menu.style.display = 'none'; refreshStats();
});

// ── Reset ────────────────────────────────────────────────────────────────────
document.getElementById('reset').onclick = () => {
  cy.elements().remove();
  cy.add(JSON.parse(JSON.stringify(ORIGINAL)));
  cy.style().update();
  typesDiv.querySelectorAll('input[data-type]').forEach(c => c.checked = true);
  applyLayered();
  refreshStats();
};

// ── Export ───────────────────────────────────────────────────────────────────
function fname(ext) {
  let n = (document.getElementById('fname').value || 'neighbors_graph').trim();
  return n.toLowerCase().endsWith('.' + ext) ? n : n + '.' + ext;
}
function download(blobUrl, name) {
  const a = document.createElement('a'); a.href = blobUrl; a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
}
document.getElementById('png').onclick = () => {
  const uri = cy.png({ full: true, scale: 2, bg: '#ffffff' });
  download(uri, fname('png'));
};
document.getElementById('pdf').onclick = () => {
  // Print the FULL graph (not just the on-screen viewport): render it to an
  // image like the PNG export, drop it in a new tab, and print that.
  const uri = cy.png({ full: true, scale: 2, bg: '#ffffff' });
  const name = (document.getElementById('fname').value || 'neighbors_graph').trim();
  const w = window.open('', '_blank');
  if (!w) { alert('Popup blocked — allow pop-ups for this page, or use Download PNG.'); return; }
  w.document.write('<title>' + name + '</title>' +
    '<style>@page{margin:8mm} html,body{margin:0} img{width:100%;height:auto}</style>' +
    '<img src="' + uri + '" onload="setTimeout(function(){window.focus();window.print();},300)">');
  w.document.close();
};

// draw.io / diagrams.net XML (mxGraph) of the CURRENTLY VISIBLE graph, keeping
// each node's position, colour and full name. Open it in app.diagrams.net via
// File > Open (or Extras > Edit Diagram) to edit it as real shapes.
function drawioXML() {
  const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;')
                            .replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  const vn = cy.nodes(':visible');
  let minX = Infinity, minY = Infinity;
  vn.forEach(n => { const p = n.position(); minX = Math.min(minX, p.x); minY = Math.min(minY, p.y); });
  if (!isFinite(minX)) { minX = 0; minY = 0; }
  const cells = ['<mxCell id="0"/>', '<mxCell id="1" parent="0"/>'];
  const idmap = {}; let k = 2;
  vn.forEach(n => {
    const id = 'n' + (k++); idmap[n.id()] = id;
    const p = n.position(), w = 160, h = 40;
    const style = 'rounded=1;whiteSpace=wrap;html=1;fillColor=' + n.data('color') +
                  ';strokeColor=#333333;fontSize=10;';
    const val = n.data('alias') ? n.data('label') : n.id();   // alias, else full name
    cells.push('<mxCell id="' + id + '" value="' + esc(val) + '" style="' + style +
      '" vertex="1" parent="1"><mxGeometry x="' + Math.round(p.x - minX) + '" y="' +
      Math.round(p.y - minY) + '" width="' + w + '" height="' + h + '" as="geometry"/></mxCell>');
  });
  cy.edges(':visible').forEach(ed => {
    const s = idmap[ed.source().id()], t = idmap[ed.target().id()];
    if (!s || !t) return;
    cells.push('<mxCell id="e' + (k++) + '" style="endArrow=classic;html=1;rounded=0;" ' +
      'edge="1" parent="1" source="' + s + '" target="' + t + '"><mxGeometry relative="1" as="geometry"/></mxCell>');
  });
  return '<mxfile><diagram name="neighbours" id="g1"><mxGraphModel grid="1" gridSize="10" ' +
    'arrows="1" fold="1" page="1" pageScale="1" math="0" shadow="0"><root>' +
    cells.join('') + '</root></mxGraphModel></diagram></mxfile>';
}
document.getElementById('drawio').onclick = () => {
  download('data:application/xml;charset=utf-8,' + encodeURIComponent(drawioXML()), fname('xml'));
};

// ── Save / load arrangement config ───────────────────────────────────────────
// Captures how you arranged the graph so you can restore it on the same or a
// similar file: node positions, which types/nodes are hidden, deletions, view.
function saveConfig() {
  const positions = {};
  cy.nodes().forEach(n => {
    const p = n.position(); positions[n.id()] = { x: Math.round(p.x), y: Math.round(p.y) };
  });
  const hiddenTypes = [...typesDiv.querySelectorAll('input[data-type]')]
      .filter(c => !c.checked).map(c => c.dataset.type);
  const hiddenSet = new Set(hiddenTypes);
  const hiddenNodes = cy.nodes()
      .filter(n => n.style('display') === 'none' && !hiddenSet.has(n.data('type')))
      .map(n => n.id());
  const present = new Set(cy.nodes().map(n => n.id()));
  const deletedNodes = Object.keys(PRESET_POS).filter(id => !present.has(id));
  const aliases = {};
  cy.nodes().forEach(n => { if (n.data('alias')) aliases[n.id()] = n.data('alias'); });
  const cfg = { version: 1, positions, hiddenTypes, hiddenNodes, deletedNodes, aliases,
                pan: cy.pan(), zoom: cy.zoom() };
  download('data:application/json;charset=utf-8,' + encodeURIComponent(JSON.stringify(cfg)),
           fname('json'));
}
function applyConfig(cfg) {
  if (!cfg) return;
  (cfg.deletedNodes || []).forEach(id => { const n = cy.$id(id); if (n.length) n.remove(); });
  const P = cfg.positions || {};
  cy.nodes().forEach(n => { if (P[n.id()]) n.position(P[n.id()]); });
  const A = cfg.aliases || {};
  cy.nodes().forEach(n => {
    if (A[n.id()] !== undefined) { n.data('alias', A[n.id()]); n.data('label', A[n.id()] || defLabel(n.id())); }
  });
  const hidden = new Set(cfg.hiddenTypes || []);
  typesDiv.querySelectorAll('input[data-type]').forEach(c => {
    c.checked = !hidden.has(c.dataset.type);
    applyType(c.dataset.type, c.checked);
  });
  (cfg.hiddenNodes || []).forEach(id => { const n = cy.$id(id); if (n.length) n.style('display', 'none'); });
  if (cfg.zoom) cy.zoom(cfg.zoom);
  if (cfg.pan) cy.pan(cfg.pan);
  refreshStats();
}
document.getElementById('saveCfg').onclick = saveConfig;
document.getElementById('loadCfg').onclick = () => document.getElementById('cfgFile').click();
document.getElementById('cfgFile').onchange = e => {
  const f = e.target.files[0]; if (!f) return;
  const r = new FileReader();
  r.onload = () => { try { applyConfig(JSON.parse(r.result)); }
                     catch (err) { alert('Invalid config file: ' + err); } };
  r.readAsText(f);
  e.target.value = '';   // allow re-loading the same file
};
</script>
</body>
</html>
"""


def build_html(file_path, node_type, edges):
    elements = build_elements(node_type, edges)
    default_name = os.path.splitext(os.path.basename(file_path))[0].replace("_0", "")
    html = (HTML_TEMPLATE
            .replace("__TITLE__", "e-MDB neighbours — " + os.path.basename(file_path))
            .replace("__ELEMENTS_JSON__", json.dumps(elements))
            .replace("__TYPES_JSON__", json.dumps(type_summary(node_type)))
            .replace("__DEFAULT_FILENAME__", default_name or "neighbors_graph"))
    return html


def main():
    ap = argparse.ArgumentParser(description="Interactive HTML view of a neighbors_full.txt file.")
    ap.add_argument("-f", "--file", required=True, help="Path to neighbors_full_N.txt")
    ap.add_argument("-o", "--out", default=None, help="Output .html path (default: alongside the input).")
    ap.add_argument("--open", action="store_true", help="Open the HTML in a browser when done.")
    args = ap.parse_args()

    if not os.path.isfile(args.file):
        print(f"Error: file not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    node_type, edges = parse(args.file)
    print(f"Parsed {len(node_type)} nodes, {len(edges)} edges from {os.path.basename(args.file)}")

    out = args.out or (os.path.splitext(args.file)[0] + "_graph.html")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(build_html(args.file, node_type, edges))
    print(f"Interactive graph written to: {out}")
    print("Open it in a browser. Toggle node types, right-click nodes to hide/delete, "
          "and use Export for PNG / PDF / CSV.")

    if args.open:
        import webbrowser
        webbrowser.open("file://" + os.path.abspath(out))


if __name__ == "__main__":
    main()
