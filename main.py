"""
Dynamic Pathfinding Agent — Assignment Submission
==================================================
Algorithms : A* Search  |  Greedy Best-First Search (GBFS)
Heuristics : Manhattan Distance  |  Euclidean Distance
GUI        : Tkinter (built-in — no pip install needed)
Python     : 3.8+

Run:
    python main.py

Controls:
    Mode buttons     → Switch between Wall / Start / Goal edit mode
    Click on grid    → Place wall (Wall mode) / Move start or goal
    Right-click      → Erase wall
    Click + Drag     → Draw walls continuously
    Algorithm toggle → A* or GBFS
    Heuristic toggle → Manhattan or Euclidean
    ▶ Run            → Full animated search
    Step             → Advance one frame at a time
    Clear            → Clear visualisation, keep walls
    Reset            → Wipe entire grid
    Gen Maze         → Random obstacle maze
    Dynamic Mode     → Obstacles spawn mid-search; auto re-plans
"""

import tkinter as tk
from collections import defaultdict
import heapq, math, random, time

# ══════════════════════════════════════════════════════════
#  COLOUR PALETTE
# ══════════════════════════════════════════════════════════
P = {
    "bg":       "#0b0c17",
    "panel":    "#10121e",
    "grid":     "#080910",
    "wall":     "#1c1f31",
    "start":    "#00e676",
    "goal":     "#ff1744",
    "frontier": "#ffd600",
    "visited":  "#1565c0",
    "path":     "#ce93d8",
    "text":     "#e0e4ff",
    "sub":      "#4a5568",
    "border":   "#1a1d2e",
    "btn":      "#161928",
    "btn_h":    "#1e2440",
    "btn_sel":  "#0d2a6e",
    "g_btn":    "#0a3d1a",
    "r_btn":    "#5c0a0a",
    "o_btn":    "#4a2000",
    "astar_col":"#283593",
    "gbfs_col": "#33691e",
}

# ══════════════════════════════════════════════════════════
#  HEURISTICS
# ══════════════════════════════════════════════════════════
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# ══════════════════════════════════════════════════════════
#  PATH RECONSTRUCTION
# ══════════════════════════════════════════════════════════
def reconstruct(came_from, goal):
    path, n = [], goal
    while n is not None:
        path.append(n)
        n = came_from.get(n)
    path.reverse()
    return path

# ══════════════════════════════════════════════════════════
#  SEARCH ALGORITHMS
# ══════════════════════════════════════════════════════════
def run_astar(grid, rows, cols, start, goal, hfn):
    """
    A* Search  —  f(n) = g(n) + h(n)
    Uses an Expanded List (closed set). Nodes can be re-opened
    if a cheaper path is found, guaranteeing optimality when
    the heuristic is admissible.
    Returns: (path, visited_in_order)
    """
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    open_heap = [(hfn(start, goal), 0, start)]
    came_from = {start: None}
    visited   = []
    closed    = set()

    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur not in (start, goal):
            visited.append(cur)
        if cur == goal:
            return reconstruct(came_from, goal), visited
        r, c = cur
        for nr, nc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
            nb = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1:
                ng = g_score[cur] + 1
                if ng < g_score[nb]:
                    g_score[nb]   = ng
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (ng + hfn(nb, goal), ng, nb))
    return None, visited


def run_gbfs(grid, rows, cols, start, goal, hfn):
    """
    Greedy Best-First Search  —  f(n) = h(n) only
    Uses a Strict Visited List. Nodes are never re-visited,
    making it fast but NOT guaranteed to find the optimal path.
    Returns: (path, visited_in_order)
    """
    open_heap = [(hfn(start, goal), start)]
    came_from = {start: None}
    visited   = []
    closed    = set()

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur not in (start, goal):
            visited.append(cur)
        if cur == goal:
            return reconstruct(came_from, goal), visited
        r, c = cur
        for nr, nc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
            nb = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1:
                if nb not in closed and nb not in came_from:
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (hfn(nb, goal), nb))
    return None, visited

# ══════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ══════════════════════════════════════════════════════════
class App:
    BATCH    = 4      # cells animated per frame
    DYN_PROB = 0.055  # probability of spawning obstacle per frame

    def __init__(self, root):
        self.root = root
        root.title("Dynamic Pathfinding Agent  |  A* & GBFS")
        root.configure(bg=P["bg"])
        root.resizable(False, False)

        # ── grid setup ───────────────────────
        self.ROWS = 22
        self.COLS = 40
        self.CS   = 24   # cell size in pixels

        self.grid  = [[0]*self.COLS for _ in range(self.ROWS)]
        self.start = (1, 1)
        self.goal  = (self.ROWS-2, self.COLS-2)
        self.grid[self.start[0]][self.start[1]] = 2
        self.grid[self.goal[0]][self.goal[1]]   = 3

        # ── ui variables ─────────────────────
        self.algo      = tk.StringVar(value="A*")
        self.heur      = tk.StringVar(value="Manhattan")
        self.edit_mode = tk.StringVar(value="Wall")
        self.dyn_on    = tk.BooleanVar(value=False)
        self.speed     = tk.IntVar(value=14)

        # ── search state ─────────────────────
        self.vis_cells   = []    # [(cell, "fwd")] ordered for animation
        self.vis_idx     = 0
        self.path        = []
        self.path_set    = set()
        self.vis_set     = set()
        self.anim_id     = None
        self.drag_val    = None

        # ── metrics ──────────────────────────
        self.sv_nodes  = tk.StringVar(value="—")
        self.sv_cost   = tk.StringVar(value="—")
        self.sv_time   = tk.StringVar(value="—")
        self.sv_status = tk.StringVar(value="Select an algorithm and press  ▶ Run")

        self._build_ui()
        self._redraw_all()

    # ════════════════════ GRID HELPERS ═══════════════════
    def _neighbors(self, r, c):
        out = []
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < self.ROWS and 0 <= nc < self.COLS and self.grid[nr][nc] != 1:
                out.append((nr, nc))
        return out

    def _hfn(self):
        return manhattan if self.heur.get() == "Manhattan" else euclidean

    def _cancel(self):
        if self.anim_id:
            self.root.after_cancel(self.anim_id)
            self.anim_id = None

    def _clear_state(self):
        self._cancel()
        self.vis_cells = []; self.vis_idx = 0
        self.path = []; self.path_set = set(); self.vis_set = set()

    # ════════════════════ UI CONSTRUCTION ════════════════
    def _build_ui(self):
        # ── top control panel
        top = tk.Frame(self.root, bg=P["panel"])
        top.pack(fill=tk.X)
        self._build_top(top)

        # ── grid canvas
        cf = tk.Frame(self.root, bg=P["bg"], padx=4, pady=4)
        cf.pack()
        self.canvas = tk.Canvas(cf,
                                width =self.COLS*self.CS+1,
                                height=self.ROWS*self.CS+1,
                                bg=P["grid"],
                                highlightthickness=1,
                                highlightbackground=P["border"],
                                cursor="crosshair")
        self.canvas.pack()
        self.canvas.bind("<Button-1>",        self._on_click)
        self.canvas.bind("<Button-3>",        self._on_rclick)
        self.canvas.bind("<B1-Motion>",       self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda e: setattr(self,"drag_val",None))

        # ── bottom metrics panel
        bot = tk.Frame(self.root, bg=P["panel"])
        bot.pack(fill=tk.X)
        self._build_bottom(bot)

    def _build_top(self, p):
        # ── Row 1: algorithm + heuristic + mode + action buttons
        r1 = tk.Frame(p, bg=P["panel"])
        r1.pack(fill=tk.X, padx=8, pady=(6,3))

        def sep(): tk.Frame(r1, bg=P["border"], width=1, height=24).pack(side=tk.LEFT, padx=8)
        def lbl(t): tk.Label(r1, text=t, bg=P["panel"], fg=P["sub"],
                             font=("Courier",8,"bold")).pack(side=tk.LEFT, padx=(0,4))

        # Algorithm toggle
        lbl("ALGORITHM:")
        for name, fg, sel in (("A*", P["astar_col"], P["btn_sel"]),
                               ("GBFS", P["gbfs_col"], "#1a3d0a")):
            tk.Radiobutton(r1, text=name, variable=self.algo, value=name,
                           indicatoron=False, selectcolor=sel,
                           bg=P["btn"], fg=fg,
                           activebackground=P["btn_h"], activeforeground=P["text"],
                           font=("Courier",11,"bold"),
                           relief="flat", bd=0, padx=10, pady=4,
                           cursor="hand2").pack(side=tk.LEFT, padx=2)

        sep()

        # Heuristic toggle
        lbl("HEURISTIC:")
        for h in ("Manhattan", "Euclidean"):
            tk.Radiobutton(r1, text=h, variable=self.heur, value=h,
                           indicatoron=False, selectcolor=P["btn_sel"],
                           bg=P["btn"], fg=P["text"],
                           activebackground=P["btn_h"], activeforeground=P["text"],
                           font=("Courier",9,"bold"),
                           relief="flat", bd=0, padx=7, pady=4,
                           cursor="hand2").pack(side=tk.LEFT, padx=2)

        sep()

        # Edit mode
        lbl("EDIT:")
        for m, sel in (("Wall",P["btn_sel"]),("Start",P["g_btn"]),("Goal",P["r_btn"])):
            tk.Radiobutton(r1, text=m, variable=self.edit_mode, value=m,
                           indicatoron=False, selectcolor=sel,
                           bg=P["btn"], fg=P["text"],
                           activebackground=P["btn_h"], activeforeground=P["text"],
                           font=("Courier",9,"bold"),
                           relief="flat", bd=0, padx=7, pady=4,
                           cursor="hand2").pack(side=tk.LEFT, padx=2)

        sep()

        # Action buttons
        for txt, cmd, bg in [
            ("▶  Run",     self._run,       P["g_btn"]),
            ("Step",       self._step,      P["btn"]),
            ("Clear",      self._clear_vis, P["o_btn"]),
            ("Reset",      self._reset,     P["r_btn"]),
            ("Gen Maze",   self._gen_maze,  P["btn"]),
        ]:
            b = tk.Button(r1, text=txt, command=cmd,
                          bg=bg, fg=P["text"],
                          activebackground=P["btn_h"], activeforeground=P["text"],
                          font=("Courier",9,"bold"), relief="flat", bd=0,
                          padx=8, pady=4, cursor="hand2")
            b.pack(side=tk.LEFT, padx=2)

        sep()

        # Dynamic mode + speed
        tk.Checkbutton(r1, text="Dynamic Mode", variable=self.dyn_on,
                       bg=P["panel"], fg=P["text"], selectcolor=P["g_btn"],
                       activebackground=P["panel"], activeforeground=P["text"],
                       font=("Courier",9,"bold")).pack(side=tk.LEFT, padx=4)

        lbl("  Speed:")
        tk.Scale(r1, variable=self.speed, from_=1, to=80,
                 orient=tk.HORIZONTAL, length=80,
                 bg=P["panel"], fg=P["text"], troughcolor=P["btn"],
                 highlightthickness=0, showvalue=False,
                 sliderlength=14).pack(side=tk.LEFT)

    def _build_bottom(self, p):
        row = tk.Frame(p, bg=P["panel"])
        row.pack(fill=tk.X, padx=8, pady=5)

        # Metric boxes
        for label, var, col in [
            ("NODES EXP.", self.sv_nodes, "#ffd600"),
            ("PATH COST",  self.sv_cost,  "#00e676"),
            ("TIME (ms)",  self.sv_time,  "#40c4ff"),
        ]:
            f = tk.Frame(row, bg=P["panel"])
            f.pack(side=tk.LEFT, padx=14)
            tk.Label(f, text=label, bg=P["panel"], fg=P["sub"],
                     font=("Courier",7,"bold")).pack()
            tk.Label(f, textvariable=var, bg=P["panel"], fg=col,
                     font=("Courier",13,"bold")).pack()

        # Status message
        tk.Label(row, textvariable=self.sv_status, bg=P["panel"], fg=P["text"],
                 font=("Courier",9), wraplength=620, justify="left",
                 anchor="w").pack(side=tk.LEFT, padx=20, fill=tk.X, expand=True)

        # Colour legend
        lf = tk.Frame(row, bg=P["panel"])
        lf.pack(side=tk.RIGHT, padx=6)
        for col, name in [
            (P["start"],    "Start"),
            (P["goal"],     "Goal"),
            (P["wall"],     "Wall"),
            (P["frontier"], "Frontier"),
            (P["visited"],  "Visited"),
            (P["path"],     "Path"),
        ]:
            f = tk.Frame(lf, bg=P["panel"])
            f.pack(side=tk.LEFT, padx=3)
            tk.Canvas(f, width=10, height=10, bg=col,
                      highlightthickness=0).pack(side=tk.LEFT, padx=(0,2))
            tk.Label(f, text=name, bg=P["panel"], fg=P["sub"],
                     font=("Courier",7)).pack(side=tk.LEFT)

    # ════════════════════ CANVAS DRAWING ═════════════════
    def _color(self, r, c):
        v, pos = self.grid[r][c], (r, c)
        if v == 2:                   return P["start"]
        if v == 3:                   return P["goal"]
        if v == 1:                   return P["wall"]
        if pos in self.path_set:     return P["path"]
        if pos in self.vis_set:      return P["visited"]
        return P["grid"]

    def _redraw_all(self):
        self.canvas.delete("all")
        cs = self.CS
        for r in range(self.ROWS):
            for c in range(self.COLS):
                x, y = c*cs, r*cs
                self.canvas.create_rectangle(
                    x+1, y+1, x+cs-1, y+cs-1,
                    fill=self._color(r, c),
                    outline=P["border"], width=1,
                    tags=f"C{r}_{c}")
        self._draw_label(*self.start, "S")
        self._draw_label(*self.goal,  "G")

    def _draw_label(self, r, c, text):
        cs = self.CS
        self.canvas.create_text(
            c*cs + cs//2, r*cs + cs//2,
            text=text, fill="white",
            font=("Courier", int(cs*.55), "bold"),
            tags=f"L{r}_{c}")

    def _repaint(self, r, c):
        self.canvas.itemconfig(f"C{r}_{c}", fill=self._color(r, c))
        self.canvas.delete(f"L{r}_{c}")
        if (r,c) == self.start: self._draw_label(r, c, "S")
        if (r,c) == self.goal:  self._draw_label(r, c, "G")

    # ════════════════════ MOUSE EVENTS ═══════════════════
    def _gpos(self, event):
        c = event.x // self.CS
        r = event.y // self.CS
        if 0 <= r < self.ROWS and 0 <= c < self.COLS:
            return r, c
        return None

    def _on_click(self, event):
        pos = self._gpos(event)
        if pos is None: return
        r, c = pos
        mode = self.edit_mode.get()

        if mode == "Start":
            sr, sc = self.start
            self.grid[sr][sc] = 0;  self._repaint(sr, sc)
            self.start = (r, c)
            self.grid[r][c] = 2;    self._repaint(r, c)
            self._clear_state()
            self.sv_status.set(f"Start moved to ({r}, {c})")

        elif mode == "Goal":
            gr, gc = self.goal
            self.grid[gr][gc] = 0;  self._repaint(gr, gc)
            self.goal = (r, c)
            self.grid[r][c] = 3;    self._repaint(r, c)
            self._clear_state()
            self.sv_status.set(f"Goal moved to ({r}, {c})")

        elif mode == "Wall":
            if pos not in (self.start, self.goal):
                new_v = 1 if self.grid[r][c] != 1 else 0
                self.drag_val = new_v
                self.grid[r][c] = new_v
                self._repaint(r, c)
                self._clear_state()

    def _on_rclick(self, event):
        pos = self._gpos(event)
        if pos is None: return
        r, c = pos
        if self.grid[r][c] == 1:
            self.grid[r][c] = 0
            self._repaint(r, c)

    def _on_drag(self, event):
        if self.edit_mode.get() != "Wall" or self.drag_val is None: return
        pos = self._gpos(event)
        if pos is None: return
        r, c = pos
        if pos not in (self.start, self.goal) and self.grid[r][c] != self.drag_val:
            self.grid[r][c] = self.drag_val
            self._repaint(r, c)

    # ════════════════════ CONTROLS ════════════════════════
    def _run(self):
        self._clear_state()
        self._redraw_all()
        result = self._execute_search()
        if result is None: return
        path, vis_cells, ms = result
        self.path      = path or []
        self.vis_cells = vis_cells
        self.vis_idx   = 0
        self.sv_nodes.set(str(len(vis_cells)))
        self.sv_cost.set(str(len(path)-1) if path else "∞")
        self.sv_time.set(f"{ms}")
        algo = self.algo.get()
        heur = self.heur.get()[:4]
        self.sv_status.set(
            f"{'✓ Path found' if path else '✗ No path found'}  |  "
            f"Algorithm: {algo}  |  Heuristic: {heur}  |  "
            f"Nodes expanded: {len(vis_cells)}  |  "
            f"Path cost: {len(path)-1 if path else '∞'}  |  "
            f"Time: {ms} ms")
        self._tick()

    def _step(self):
        if not self.vis_cells:
            self._clear_state()
            self._redraw_all()
            result = self._execute_search()
            if result is None: return
            path, vis_cells, ms = result
            self.path = path or []
            self.vis_cells = vis_cells
            self.vis_idx   = 0
            self.sv_nodes.set(str(len(vis_cells)))
            self.sv_cost.set(str(len(path)-1) if path else "∞")
            self.sv_time.set(f"{ms}")

        if self.vis_idx < len(self.vis_cells):
            cell = self.vis_cells[self.vis_idx]
            self.vis_set.add(cell)
            self._repaint(*cell)
            self.vis_idx += 1
            self.sv_status.set(
                f"Step {self.vis_idx} / {len(self.vis_cells)}  —  "
                f"use Step button to advance frame by frame")

        if self.vis_idx >= len(self.vis_cells):
            self._show_path()

    def _tick(self):
        """Animation loop — called repeatedly via root.after()"""
        if self.vis_idx < len(self.vis_cells):
            # Dynamic obstacle spawning
            if self.dyn_on.get():
                self._maybe_spawn_obstacle()
            # Draw a batch of cells per frame
            for _ in range(self.BATCH):
                if self.vis_idx < len(self.vis_cells):
                    cell = self.vis_cells[self.vis_idx]
                    self.vis_set.add(cell)
                    self._repaint(*cell)
                    self.vis_idx += 1
            delay = max(1, self.speed.get())
            self.anim_id = self.root.after(delay, self._tick)
        else:
            self._show_path()

    def _show_path(self):
        self.path_set = set(self.path)
        for pos in self.path:
            self._repaint(*pos)
        self.sv_status.set(
            self.sv_status.get().replace("searching…", "done!"))

    def _clear_vis(self):
        self._clear_state()
        self._redraw_all()
        self.sv_status.set("Visualisation cleared — walls kept.")

    def _reset(self):
        self._cancel()
        self.grid = [[0]*self.COLS for _ in range(self.ROWS)]
        self.grid[self.start[0]][self.start[1]] = 2
        self.grid[self.goal[0]][self.goal[1]]   = 3
        self._clear_state()
        self.sv_nodes.set("—"); self.sv_cost.set("—"); self.sv_time.set("—")
        self.sv_status.set("Grid reset — draw walls and press ▶ Run.")
        self._redraw_all()

    def _gen_maze(self):
        self._cancel()
        self.grid = [[0]*self.COLS for _ in range(self.ROWS)]
        for r in range(self.ROWS):
            for c in range(self.COLS):
                if (r,c) not in (self.start, self.goal) and random.random() < 0.28:
                    self.grid[r][c] = 1
        self.grid[self.start[0]][self.start[1]] = 2
        self.grid[self.goal[0]][self.goal[1]]   = 3
        self._clear_state()
        self._redraw_all()
        self.sv_status.set("Random maze generated — press ▶ Run!")

    # ════════════════════ DYNAMIC OBSTACLES ══════════════
    def _maybe_spawn_obstacle(self):
        """Randomly spawn a wall; re-plan immediately if it blocks the path."""
        if random.random() > self.DYN_PROB:
            return
        r = random.randint(0, self.ROWS-1)
        c = random.randint(0, self.COLS-1)
        if (r,c) in (self.start, self.goal) or self.grid[r][c] != 0:
            return

        self.grid[r][c] = 1
        self._repaint(r, c)

        # If the new wall is on the current path → re-plan from current agent position
        if self.path and (r,c) in set(self.path):
            self._cancel()
            agent = (self.vis_cells[max(0, self.vis_idx-1)]
                     if self.vis_cells and self.vis_idx > 0 else self.start)
            result = self._execute_search(start_override=agent)
            if result is None: return
            new_path, new_vis, ms = result
            if new_path:
                self.path      = new_path
                self.vis_cells = new_vis
                self.vis_idx   = 0
                self.vis_set   = set()
                self.path_set  = set()
                self.sv_nodes.set(str(int(self.sv_nodes.get() or 0) + len(new_vis)))
                self.sv_cost.set(str(len(new_path)-1))
                self.sv_status.set(
                    f"⚡ Obstacle detected! Re-planning…  "
                    f"New cost: {len(new_path)-1}  |  Time: {ms} ms")
                self.anim_id = self.root.after(max(1, self.speed.get()), self._tick)
            else:
                self.sv_status.set("⚠ Re-plan failed — no path exists!")

    # ════════════════════ SEARCH DISPATCH ════════════════
    def _execute_search(self, start_override=None):
        s   = start_override if start_override else self.start
        g   = self.goal
        hfn = self._hfn()
        t0  = time.perf_counter()

        try:
            if self.algo.get() == "A*":
                path, vis = run_astar(self.grid, self.ROWS, self.COLS, s, g, hfn)
            else:
                path, vis = run_gbfs(self.grid, self.ROWS, self.COLS, s, g, hfn)
        except Exception as e:
            self.sv_status.set(f"Error: {e}")
            return None

        ms = round((time.perf_counter()-t0)*1000, 2)
        return path, vis, ms


# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
