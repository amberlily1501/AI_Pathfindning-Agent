# Dynamic Pathfinding Agent
### AI Assignment 2 — Informed Search Algorithms

A real-time grid-based pathfinding visualizer built with **Python + Tkinter**.  
Implements **A\* Search** and **Greedy Best-First Search (GBFS)** with dynamic obstacle support.

---

## How to Run

**No installation required.** Tkinter is built into Python.

```bash
python main.py
```

> If you get a tkinter error on Linux, run: `sudo apt install python3-tk`

**Requirements:** Python 3.8 or higher

---

## Features

| Feature | Description |
|---|---|
| **A\* Search** | `f(n) = g(n) + h(n)` — Optimal path guaranteed |
| **Greedy BFS** | `f(n) = h(n)` — Fast but not always optimal |
| **Manhattan Distance** | `|x1-x2| + |y1-y2|` — Grid heuristic |
| **Euclidean Distance** | `√((x1-x2)² + (y1-y2)²)` — Straight-line heuristic |
| **Dynamic Mode** | Obstacles spawn mid-search; agent re-plans instantly |
| **Interactive Editor** | Click/drag to draw walls, set custom Start & Goal |
| **Maze Generator** | Random obstacle generation (≈28% density) |
| **Step Mode** | Advance the search one frame at a time |
| **Metrics Dashboard** | Nodes expanded, path cost, execution time (ms) |

---

## Controls

| Action | How |
|---|---|
| **Draw walls** | Click or click+drag on grid (Wall mode) |
| **Erase walls** | Right-click on a wall |
| **Move Start** | Select "Start" mode → click any cell |
| **Move Goal** | Select "Goal" mode → click any cell |
| **Run search** | Press **▶ Run** button |
| **Step through** | Press **Step** button repeatedly |
| **Clear path** | Press **Clear** (keeps walls) |
| **Reset everything** | Press **Reset** |
| **Generate maze** | Press **Gen Maze** |
| **Toggle dynamic obstacles** | Check **Dynamic Mode** checkbox |
| **Adjust speed** | Drag the Speed slider |

---

## Colour Legend

| Colour | Meaning |
|---|---|
| 🟢 Green | Start node |
| 🔴 Red | Goal node |
| ⬛ Dark blue | Wall / obstacle |
| 🟡 Yellow | Frontier (nodes in priority queue) |
| 🔵 Blue | Visited / expanded nodes |
| 🟣 Purple | Final path |

---

## Algorithm Details

### A\* Search
- Evaluation function: **f(n) = g(n) + h(n)**
- `g(n)` = actual cost from start to node n
- `h(n)` = heuristic estimate from n to goal
- Uses an **Expanded List** (closed set) — nodes can be re-opened if a cheaper path is found
- **Guarantees the optimal path** when heuristic is admissible
- Expands more nodes than GBFS but the result is always optimal

### Greedy Best-First Search (GBFS)
- Evaluation function: **f(n) = h(n)** only
- Uses a **Strict Visited List** — nodes are never revisited
- Greedily moves toward the goal based purely on heuristic
- **Not guaranteed to find the optimal path**
- Typically faster than A\* (fewer nodes expanded) but may find longer routes

### Dynamic Re-planning
When **Dynamic Mode** is enabled:
1. New wall obstacles randomly spawn during the search animation
2. If a spawned obstacle **blocks the current path** → agent immediately re-plans from its current position
3. If the obstacle is **off the current path** → it is placed but no re-planning occurs (efficient)

---

## Pros & Cons (Experimental Findings)

### A\*
**Pros:**
- Always finds the shortest path (optimal)
- Heuristic guidance makes it much faster than uninformed search
- Euclidean heuristic reduces nodes explored further

**Cons:**
- Expands more nodes than GBFS to guarantee optimality
- Slower than GBFS in terms of raw speed

### GBFS
**Pros:**
- Very fast — reaches the goal quickly
- Expands fewer nodes in open/easy maps
- Good for applications where speed matters more than optimality

**Cons:**
- Can find suboptimal (longer) paths
- Susceptible to the "U-shaped wall" trap — may struggle badly in mazes
- With a Strict Visited List, cannot recover from wrong early decisions

---

## Test Cases (for Report Screenshots)

### Best Case — A\*
Open grid with no walls, Start at top-left, Goal at bottom-right.  
A\* finds the optimal diagonal-ish path with minimal expansion.

### Worst Case — A\*
Dense random maze. A\* must explore a large portion of the grid before finding the path.

### Best Case — GBFS
Open grid, goal visible in a straight line.  
GBFS zooms directly to the goal, expanding very few nodes.

### Worst Case — GBFS
U-shaped wall between Start and Goal.  
GBFS heads straight for the goal, gets trapped, and wanders extensively before finding a way around.

---

## File Structure

```
AI_A2_XXF-YYYY/
│
├── main.py       ← Complete source code (single file)
└── README.md     ← This file
```

---

## Dependencies

None. Uses only Python standard library:
- `tkinter` — GUI
- `heapq` — Priority queue for A\* and GBFS
- `math` — Euclidean distance
- `random` — Maze generation and dynamic obstacles
- `time` — Execution time measurement
- `collections` — defaultdict for g-scores
