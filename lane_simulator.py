#!/usr/bin/env python3
# lane_simulator.py — final stable version with:
# - node-based OR xy-based lights
# - invisible trigger areas
# - directional stopping logic
# - smooth motion / no slowing bug
# - supports your current map format exactly

import sys, json, math, random, time, heapq, os
import pygame
from pygame.math import Vector2

W, H = 1000, 700
BG = (230, 235, 240)
ROAD = (40, 40, 40)
HUBC = (255, 200, 0)
CARC = (30, 60, 200)
LIGHTG = (0, 180, 0)
LIGHTR = (180, 0, 0)
JOINC = (255, 119, 0)

# Tunables
SPAWN_SAFE = 14
SPAWN_DIV = 3.0
STUCK_TIMEOUT = 6.0
JOIN_RADIUS = 40.0
DEBUG = False

DEFAULT_DESIRED_SPEED = 80.0
SPEED_VARIATION = 16.0
ACCEL = 150.0
MAX_BRAKE = 250.0
MIN_CRUISE = 12.0
STOP_DISTANCE = 12.0
CAUTION_DISTANCE = 40.0


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dijkstra(edges, start, goal):
    if start == goal:
        return [start]
    if start not in edges or goal not in edges:
        return None

    distmap = {start: 0.0}
    prev = {}
    pq = []
    c = 0
    heapq.heappush(pq, (0.0, c, start))
    visited = set()

    while pq:
        cd, _, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)

        if u == goal:
            break

        for v, w in edges.get(u, []):
            nd = cd + (w if w is not None else 1.0)
            if v not in distmap or nd < distmap[v]:
                distmap[v] = nd
                prev[v] = u
                c += 1
                heapq.heappush(pq, (nd, c, v))

    if goal not in distmap:
        return None

    path = []
    cur = goal
    while True:
        path.append(cur)
        if cur == start:
            break
        cur = prev[cur]
    path.reverse()
    return path


# -------------------------------------------------------------------
# LIGHT CLASS (supports node-based and x,y-based definitions)
# -------------------------------------------------------------------
class Light:
    def __init__(self, data):
        # Accept node-based or coordinate-based
        self.node = data.get("node", None)

        if self.node is not None:
            self.x = None
            self.y = None
        else:
            self.x = data.get("x")
            self.y = data.get("y")

        self.green = data.get("green", 6)
        self.red = data.get("red", 6)
        self.t = data.get("offset", 0)

        self.area = data.get("area", [])
        self.center = (self.x, self.y) if self.x is not None else None

    def bind_to_graph(self, nodes):
        """When map uses node-based lights, resolve their coordinates now."""
        if self.node is not None and self.node in nodes:
            self.x, self.y = nodes[self.node]
            self.center = (self.x, self.y)

    def update(self, dt):
        self.t += dt

    def is_green(self):
        return (self.t % (self.green + self.red)) < self.green

    # point-in-polygon check
    def contains(self, px, py):
        """
        Safe point-in-polygon test.
        Prevents crashes when area is {}, None, malformed, or too small.
        """

        poly = self.area

        # Reject if empty / None
        if not poly:
            return False

        # If the editor accidentally saved the area as a dict, convert to list
        if isinstance(poly, dict):
            # Attempt to recover a polygon from values()
            poly = list(poly.values())

        # Ensure it is a list of coordinate pairs
        if not isinstance(poly, list):
            return False

        # Ensure each element is a valid (x,y) pair
        clean_poly = []
        for p in poly:
            if isinstance(p, (list, tuple)) and len(p) == 2:
                clean_poly.append((float(p[0]), float(p[1])))

        # Need at least 3 points to form a polygon
        if len(clean_poly) < 3:
            return False

        # Standard point-in-polygon (ray casting)
        inside = False
        j = len(clean_poly) - 1

        for i in range(len(clean_poly)):
            xi, yi = clean_poly[i]
            xj, yj = clean_poly[j]

            intersect = (
                ((yi > py) != (yj > py)) and
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-6) + xi)
            )

            if intersect:
                inside = not inside

            j = i

        return inside



# -------------------------------------------------------------------
# CAR CLASS (smooth acceleration, no random slowing down)
# -------------------------------------------------------------------
class Car:
    def __init__(self, path, posmap, sim, cid):
        self.path = path
        self.posmap = posmap
        self.sim = sim
        self.id = cid
        self.edge = 0
        self.progress = 0.0

        self.desired_speed = DEFAULT_DESIRED_SPEED + random.uniform(
            -SPEED_VARIATION, SPEED_VARIATION
        )
        self.curr_speed = self.desired_speed

        self.radius = 6
        self.finished = False

        head = self.path[0]
        self.pos = Vector2(self.posmap[head])
        self._last_pos = Vector2(self.pos)
        self._stuck = 0.0

    def update(self, dt):
        if self.finished:
            return
        if self.edge + 1 >= len(self.path):
            self.finished = True
            return

        a = Vector2(self.posmap[self.path[self.edge]])
        b = Vector2(self.posmap[self.path[self.edge + 1]])
        seg = b - a
        seg_len = seg.length()

        # ZERO-LENGTH SEGMENT FIX
        if seg_len < 1e-6:
            # auto-skip this segment safely
            self.edge += 1
            if self.edge + 1 >= len(self.path):
                self.pos = Vector2(self.posmap[self.path[-1]])
                self.finished = True
                return
            # force snap to next position
            self.progress = 0.0
            self.pos = Vector2(self.posmap[self.path[self.edge]])
            return  # skip movement this frame

        # Safe normalization
        dirv = seg / seg_len


        # leader detection
        leader = None
        leader_gap = 1e9
        for o in self.sim.cars:
            if o is self or o.finished:
                continue
            if o.edge == self.edge:
                g = (o.progress - self.progress) * seg_len
                if 0 < g < leader_gap:
                    leader = o
                    leader_gap = g

        desired = self.desired_speed

        if leader:
            if leader_gap < STOP_DISTANCE:
                desired = 0
            elif leader_gap < CAUTION_DISTANCE:
                factor = (leader_gap - STOP_DISTANCE) / max(
                    1, (CAUTION_DISTANCE - STOP_DISTANCE)
                )
                factor = max(0.0, min(1.0, factor))
                desired = leader.curr_speed * (0.5 + 0.5 * factor)
            else:
                desired = min(desired, leader.curr_speed + 10)

        # light trigger control
        for L in self.sim.lights:
            if not L.is_green():
                if L.contains(self.pos.x, self.pos.y):
                    desired = 0

        # speed change
        dv = desired - self.curr_speed
        if dv > 0:
            dv = min(dv, ACCEL * dt)
        else:
            dv = max(dv, -MAX_BRAKE * dt)

        self.curr_speed += dv

        if self.curr_speed > 0 and self.curr_speed < MIN_CRUISE:
            self.curr_speed = MIN_CRUISE

        move = self.curr_speed * dt
        dp = move / seg_len
        self.progress += dp

        # stuck detection
        moved = (self.pos - self._last_pos).length()
        if moved < 0.15:
            self._stuck += dt
        else:
            self._stuck = 0
            self._last_pos = Vector2(self.pos)

        if self._stuck > STUCK_TIMEOUT:
            self.finished = True
            return

        if self.progress >= 1.0:
            self.edge += 1
            if self.edge + 1 >= len(self.path):
                self.pos = Vector2(self.posmap[self.path[-1]])
                self.finished = True
                return
            self.progress = 0.0
            self.pos = Vector2(self.posmap[self.path[self.edge]])
        else:
            self.pos = a + dirv * (self.progress * seg_len)

    def draw(self, surf):
        pygame.draw.circle(surf, CARC, (int(self.pos.x), int(self.pos.y)), self.radius)


# -------------------------------------------------------------------
# GRAPH BUILDER (unchanged, supports your hubs, nodes, joins, lanes)
# -------------------------------------------------------------------
def build_graph(lanes, joins, raw_nodes, hubs):
    nodes = {}
    edges = {}
    lane_point = {}

    # lane points
    for ln in lanes:
        lid = ln["id"]
        pts = ln["points"]
        for i, p in enumerate(pts):
            nid = f"lane_{lid}_p{i}"
            nodes[nid] = (float(p[0]), float(p[1]))
            edges.setdefault(nid, [])
            lane_point[(lid, i)] = nid

    # raw nodes
    for nd in raw_nodes:
        nid = nd["id"]
        nodes[nid] = (float(nd["x"]), float(nd["y"]))
        edges.setdefault(nid, [])

    # hub centers
    for h in hubs:
        hid = h["id"]
        nodes.setdefault(hid, (float(h["x"]), float(h["y"])))
        edges.setdefault(hid, [])

    # connect lane segments
    for ln in lanes:
        lid = ln["id"]
        pts = ln["points"]
        d = ln.get("dir", "both")
        for i in range(len(pts) - 1):
            u = lane_point[(lid, i)]
            v = lane_point[(lid, i + 1)]
            w = dist(nodes[u], nodes[v])
            if d in ("both", "A->B"):
                edges[u].append((v, w))
            if d in ("both", "B->A"):
                edges[v].append((u, w))

    # start/end nodes
    for ln in lanes:
        lid = ln["id"]
        if "start_node" in ln and ln["start_node"] in nodes:
            u = ln["start_node"]
            v = lane_point[(lid, 0)]
            edges[u].append((v, dist(nodes[u], nodes[v])))
        if "end_node" in ln and ln["end_node"] in nodes:
            v = ln["end_node"]
            u = lane_point[(lid, len(ln["points"]) - 1)]
            edges[u].append((v, dist(nodes[u], nodes[v])))

    # joins
    for j in joins:
        if "node" in j and j["node"] in nodes:
            jnid = j["node"]
        else:
            jnid = f"join_{j['id']}"
            nodes[jnid] = (float(j["x"]), float(j["y"]))
            edges.setdefault(jnid, [])

        jpos = nodes[jnid]

        for ln in lanes:
            lid = ln["id"]
            pts = ln["points"]
            best = None
            bestd = 99999
            besti = None

            for i, p in enumerate(pts):
                d = dist(jpos, p)
                if d < bestd:
                    bestd = d
                    besti = i

            if besti is not None and bestd <= JOIN_RADIUS:
                lane_nid = lane_point[(lid, besti)]
                w = dist(nodes[lane_nid], jpos)
                edges[lane_nid].append((jnid, w))
                edges[jnid].append((lane_nid, w))

    # hub in/out connection
    for h in hubs:
        hid = h["id"]
        hx, hy = float(h["x"]), float(h["y"])
        for (lid, i), nid in lane_point.items():
            px, py = nodes[nid]
            if dist((hx, hy), (px, py)) <= 100:
                hub_in = f"hub_in_{hid}_{lid}_{i}"
                hub_out = f"hub_out_{hid}_{lid}_{i}"

                nodes.setdefault(hub_in, (hx, hy))
                nodes.setdefault(hub_out, (hx, hy))

                edges.setdefault(hub_in, [])
                edges.setdefault(hub_out, [])

                edges[hub_out].append((nid, dist(nodes[hub_out], nodes[nid])))
                edges[nid].append((hub_in, dist(nodes[nid], nodes[hub_in])))
                edges[hid].append((hub_out, dist(nodes[hid], nodes[hub_out])))
                edges[hub_in].append((hid, dist(nodes[hub_in], nodes[hid])))

    # complete edge lists
    for k in list(nodes.keys()):
        edges.setdefault(k, [])

    return nodes, edges


# -------------------------------------------------------------------
# SIMULATOR
# -------------------------------------------------------------------
class Simulator:
    def __init__(self, mapfile):
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Lane Simulator (final)")
        self.clock = pygame.time.Clock()

        with open(mapfile, "r") as f:
            mp = json.load(f)

        self.roads = mp.get("roads", [])
        self.raw_nodes = mp.get("nodes", [])
        self.hubs = mp.get("hubs", [])
        self.joins = mp.get("joins", [])
        self.lanes = mp.get("lanes", [])
        raw_lights = mp.get("lights", [])
        self.slow_zones = mp.get("symbols", [])

        self.nodes, self.edges = build_graph(self.lanes, self.joins, self.raw_nodes, self.hubs)

        # load lights
        self.lights = [Light(d) for d in raw_lights]
        for lt in self.lights:
            lt.bind_to_graph(self.nodes)

        self.cars = []
        self.spawn_mode = False
        self.spawn_click = []
        self.next_car_id = 1

    # hub nodes
    def hub_out_nodes(self, hub_id):
        pref = f"hub_out_{hub_id}_"
        return [n for n in self.nodes if isinstance(n, str) and n.startswith(pref)]

    def hub_in_nodes(self, hub_id):
        pref = f"hub_in_{hub_id}_"
        return [n for n in self.nodes if isinstance(n, str) and n.startswith(pref)]

    # spawning
    def spawn_car(self, start_hub, end_hub):
        out_nodes = self.hub_out_nodes(start_hub)
        if not out_nodes and start_hub in self.nodes:
            out_nodes = [start_hub]

        in_nodes = self.hub_in_nodes(end_hub)
        if not in_nodes and end_hub in self.nodes:
            in_nodes = [end_hub]

        best_path = None
        best_len = 999999

        for o in out_nodes:
            for inn in in_nodes:
                path = dijkstra(self.edges, o, inn)
                if not path:
                    continue
                L = 0.0
                for i in range(len(path) - 1):
                    L += dist(self.nodes[path[i]], self.nodes[path[i + 1]])

                if L < best_len:
                    best_len = L
                    best_path = path

        if not best_path:
            return False

        start = Vector2(self.nodes[best_path[0]])
        for c in self.cars:
            if (c.pos - start).length() < SPAWN_SAFE:
                return False

        C = Car(best_path, self.nodes, self, self.next_car_id)
        self.next_car_id += 1
        self.cars.append(C)
        return True

    def spawn_tick(self, dt):
        for h in self.hubs:
            rate = h.get("rate", 1) / SPAWN_DIV
            if random.random() < rate * dt:
                dests = [x for x in self.hubs if x["id"] != h["id"]]
                random.shuffle(dests)
                for dst in dests[:3]:
                    if self.spawn_car(h["id"], dst["id"]):
                        break

    # update
    def update(self, dt):
        for lt in self.lights:
            lt.update(dt)

        for c in list(self.cars):
            c.update(dt)
            if c.finished:
                try:
                    self.cars.remove(c)
                except:
                    pass

    # draw
    def draw(self):
        self.screen.fill(BG)

        for r in self.roads:
            pygame.draw.rect(self.screen, ROAD, (r["x"], r["y"], r["w"], r["h"]))

        for j in self.joins:
            x = j.get("x")
            y = j.get("y")
            if x is None and "node" in j and j["node"] in self.nodes:
                x, y = self.nodes[j["node"]]
            pygame.draw.rect(self.screen, JOINC, (x - 6, y - 6, 12, 12))

        for h in self.hubs:
            pygame.draw.circle(self.screen, HUBC, (int(h["x"]), int(h["y"])), 12)

        # lights
        for L in self.lights:
            if L.center:
                col = LIGHTG if L.is_green() else LIGHTR
                pygame.draw.circle(self.screen, col, (int(L.center[0]), int(L.center[1])), 8)

        for c in self.cars:
            c.draw(self.screen)

        pygame.display.flip()

    # run loop
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False

                elif e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_s:
                        self.spawn_mode = not self.spawn_mode

                elif e.type == pygame.MOUSEBUTTONDOWN and self.spawn_mode:
                    x, y = e.pos
                    for h in self.hubs:
                        if dist((x, y), (h["x"], h["y"])) < 18:
                            self.spawn_click.append(h["id"])
                            if len(self.spawn_click) == 2:
                                self.spawn_car(self.spawn_click[0], self.spawn_click[1])
                                self.spawn_click = []
                            break

            if not self.spawn_mode:
                self.spawn_tick(dt)

            self.update(dt)
            self.draw()

        pygame.quit()


def main():
    if len(sys.argv) < 2:
        print("Usage: python lane_simulator.py map.json")
        sys.exit(1)
    Simulator(sys.argv[1]).run()


if __name__ == "__main__":
    main()
