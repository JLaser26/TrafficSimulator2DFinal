#!/usr/bin/env python3
# lane_editor.py - simple editor that creates a map for lane_simulator with lights + rotatable stop areas
import tkinter as tk
from tkinter import simpledialog, filedialog, messagebox, ttk
import json, math

WIDTH, HEIGHT = 1000, 700

# stop-area default (S2)
AREA_DEPTH = 50.0
AREA_WIDTH = 32.0

def vec_sub(a,b): return (a[0]-b[0], a[1]-b[1])
def vec_add(a,b): return (a[0]+b[0], a[1]+b[1])
def vec_mul(a,s): return (a[0]*s, a[1]*s)
def vec_len(a): return math.hypot(a[0], a[1])
def vec_norm(a):
    l = vec_len(a)
    if l < 1e-6: return (0.0, 0.0)
    return (a[0]/l, a[1]/l)
def perp(u): return (-u[1], u[0])

class Editor:
    def __init__(self, master):
        self.master = master; master.title('Lane Editor (with Lights)')
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg='white')
        self.canvas.pack(side=tk.LEFT)
        toolbar = tk.Frame(master); toolbar.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Button(toolbar, text='Add Road', command=self.mode_road).pack(fill=tk.X)
        tk.Button(toolbar, text='Add Hub', command=self.mode_hub).pack(fill=tk.X)
        tk.Button(toolbar, text='Add Lane', command=self.mode_lane).pack(fill=tk.X)
        tk.Button(toolbar, text='Add Join', command=self.mode_join).pack(fill=tk.X)
        tk.Button(toolbar, text='Add Light', command=self.mode_light).pack(fill=tk.X)
        tk.Button(toolbar, text='Finish Lane', command=self.finish_lane).pack(fill=tk.X, pady=(8,0))
        tk.Button(toolbar, text='Rotate Selected Light 90°', command=self.rotate_selected_light).pack(fill=tk.X, pady=(6,0))
        tk.Button(toolbar, text='Save', command=self.save).pack(fill=tk.X, pady=6)
        tk.Button(toolbar, text='Load', command=self.load).pack(fill=tk.X)

        tk.Label(toolbar, text='Light green (s):').pack(pady=(8,0))
        self.light_green_var = tk.IntVar(value=6)
        tk.Spinbox(toolbar, from_=1, to=60, textvariable=self.light_green_var, width=5).pack()

        tk.Label(toolbar, text='Light red (s):').pack(pady=(6,0))
        self.light_red_var = tk.IntVar(value=6)
        tk.Spinbox(toolbar, from_=1, to=60, textvariable=self.light_red_var, width=5).pack()

        self.mode = 'select'; self.start = None; self.temp = None
        self.roads = []; self.hubs = []; self.joins = []; self.lanes = []; self.nodes = []; self.lights = []
        self.next_id = 1
        self.current_lane_points = []
        self.selected_light_index = None  # index in self.lights when editing/rotating

        # Bindings
        self.canvas.bind('<ButtonPress-1>', self.on_press)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<Button-3>', self.on_right_click)

    def new_id(self):
        i = self.next_id; self.next_id += 1; return i

    def mode_road(self): self.mode = 'road'
    def mode_hub(self): self.mode = 'hub'
    def mode_lane(self): self.mode = 'lane'; self.current_lane_points = []
    def mode_join(self): self.mode = 'join'
    def mode_light(self): self.mode = 'light'; self._light_stage = 0; self._light_node = None; self.selected_light_index = None

    def on_press(self, e):
        x,y = e.x, e.y
        if self.mode == 'road':
            self.start = (x,y); self.temp = self.canvas.create_rectangle(x,y,x,y, fill='black')
        elif self.mode == 'hub':
            name = simpledialog.askstring('Hub name','Name:', parent=self.master)
            if name is None: return
            rate = simpledialog.askinteger('Rate','Cars per minute:', parent=self.master, minvalue=0, initialvalue=3)
            hid = self.new_id(); r = 12
            self.canvas.create_oval(x-r, y-r, x+r, y+r, fill='gold', outline='black')
            self.canvas.create_text(x, y-18, text=name)
            # add hub plus auto-created in/out nodes (two small circles)
            rot = 0
            in_pos, out_pos = self._hub_side_positions((x,y), rot)
            nid_in = self.new_id(); nid_out = self.new_id()
            self.canvas.create_oval(in_pos[0]-6,in_pos[1]-6,in_pos[0]+6,in_pos[1]+6, fill='green', outline='black')
            self.canvas.create_text(in_pos[0], in_pos[1], text='IN', anchor='w', font=('TkDefaultFont',8))
            self.canvas.create_oval(out_pos[0]-6,out_pos[1]-6,out_pos[0]+6,out_pos[1]+6, fill='red', outline='black')
            self.canvas.create_text(out_pos[0], out_pos[1], text='OUT', anchor='w', font=('TkDefaultFont',8))
            self.hubs.append({'id':hid, 'x':x, 'y':y, 'name':name, 'rate':rate, 'rot':rot})
            # record nodes as well so lanes can connect to them (editor nodes list)
            self.nodes.append({'id': nid_in, 'x': in_pos[0], 'y': in_pos[1], 'type':'hub_in', 'hub_id':hid})
            self.nodes.append({'id': nid_out, 'x': out_pos[0], 'y': out_pos[1], 'type':'hub_out', 'hub_id':hid})
        elif self.mode == 'lane':
            self.current_lane_points.append((x,y))
            self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='blue')
            if len(self.current_lane_points) > 1:
                p1 = self.current_lane_points[-2]; p2 = self.current_lane_points[-1]
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='gray', width=6)
        elif self.mode == 'join':
            jid = self.new_id()
            self.canvas.create_rectangle(x-6, y-6, x+6, y+6, fill='orange')
            self.joins.append({'id':jid, 'x':x, 'y':y})
        elif self.mode == 'light':
            # stage 0: select node near click
            if self._light_stage == 0:
                nid = self._find_node_near(x,y)
                if nid is None:
                    messagebox.showinfo('Place Light', 'Click near an existing node/join to place light.')
                    return
                self._light_node = nid
                self._light_stage = 1
                messagebox.showinfo('Place Light', 'Now click a lane segment to attach the stop-area (the area in front of the light).')
            elif self._light_stage == 1:
                # find nearest lane segment and attach area
                seg = self._find_nearest_lane_segment(x,y)
                if seg is None:
                    messagebox.showinfo('Place Light', 'Click near a lane segment (line between lane points).')
                    return
                lane, i0, pA, pB = seg
                # compute trapezium area ahead along direction from segment A->B
                A = pA; B = pB
                seg_v = vec_sub(B, A)
                u = vec_norm(seg_v)
                n = perp(u)
                light_node_pos = self._node_pos(self._light_node)
                # place light at node position; project slightly toward segment direction to align nicely
                # compute area base near = node + u*5, far = node + u*depth
                near = vec_add(light_node_pos, vec_mul(u, 5.0))
                far  = vec_add(light_node_pos, vec_mul(u, AREA_DEPTH))
                half_w = AREA_WIDTH / 2.0
                p1 = vec_add(near, vec_mul(n, half_w * -1.0))
                p2 = vec_add(near, vec_mul(n, half_w))
                p3 = vec_add(far, vec_mul(n, half_w))
                p4 = vec_add(far, vec_mul(n, half_w * -1.0))
                area_pts = [p1, p2, p3, p4]
                lid = self.new_id()
                light = {'id':lid, 'node':self._light_node, 'area':{'points': [[float(x),float(y)] for (x,y) in area_pts], 'rot':0}, 'green': int(self.light_green_var.get()), 'red': int(self.light_red_var.get()), 'offset':0}
                self.lights.append(light)
                # draw preview on canvas
                self._draw_light_preview(len(self.lights)-1)
                self._light_stage = 0
                self._light_node = None
                messagebox.showinfo('Light', 'Light placed. You can rotate it using the Rotate button.')
        else:
            # select mode - ignore left-click
            pass

    def on_drag(self, e):
        x,y = e.x, e.y
        if self.mode == 'road' and self.temp and self.start:
            x0,y0 = self.start
            self.canvas.coords(self.temp, x0, y0, x, y)

    def on_release(self, e):
        if self.mode == 'road' and self.temp and self.start:
            x0,y0 = self.start; x1,y1 = e.x, e.y
            x = min(x0,x1); y = min(y0,y1); w = abs(x1-x0); h = abs(y1-y0)
            if w > 6 and h > 6:
                rid = self.new_id()
                self.canvas.itemconfig(self.temp, tags=(f'road{rid}',))
                self.roads.append({'id':rid, 'x':x, 'y':y, 'w':w, 'h':h, 'type':'big'})
            else:
                self.canvas.delete(self.temp)
            self.temp = None; self.start = None

    def on_right_click(self, e):
        # right click: show context — allow selecting a light to rotate by right-click
        x,y = e.x, e.y
        idx = self._find_light_near(x,y)
        if idx is not None:
            self.selected_light_index = idx
            messagebox.showinfo('Light selected', f'Light id {self.lights[idx]["id"]} selected. Use Rotate button to rotate it.')

    # helpers
    def _node_pos(self, nid):
        # search raw nodes, joins, hubs by id
        for n in self.nodes:
            if n.get('id') == nid:
                return (n['x'], n['y'])
        for j in self.joins:
            if j.get('id') == nid:
                return (j['x'], j['y'])
        for h in self.hubs:
            if h.get('id') == nid:
                return (h['x'], h['y'])
        # fallback: search by numeric id in lanes' start_node/end_node
        for ln in self.lanes:
            if ln.get('start_node') == nid:
                p = ln['points'][0]; return (p[0], p[1])
            if ln.get('end_node') == nid:
                p = ln['points'][-1]; return (p[0], p[1])
        # unknown
        return (0.0, 0.0)

    def _find_node_near(self, x,y, rad=18):
        # check hubs centers, raw nodes, joins
        for h in self.hubs:
            if math.hypot(h['x'] - x, h['y'] - y) < rad:
                return h['id']
        for n in self.nodes:
            if math.hypot(n['x'] - x, n['y'] - y) < rad:
                return n['id']
        for j in self.joins:
            if math.hypot(j['x'] - x, j['y'] - y) < rad:
                return j['id']
        return None

    def _find_nearest_lane_segment(self, x, y, maxd=30.0):
        best = None; bd = 1e9
        for ln in self.lanes:
            pts = ln.get('points', [])
            for i in range(len(pts)-1):
                A = pts[i]; B = pts[i+1]
                # distance to segment
                px, py = x, y
                ax, ay = A; bx, by = B
                vx, vy = bx - ax, by - ay
                wx, wy = px - ax, py - ay
                c = (wx*vx + wy*vy)
                vv = (vx*vx + vy*vy) or 1e-6
                t = max(0.0, min(1.0, c / vv))
                proj = (ax + vx * t, ay + vy * t)
                d = math.hypot(px - proj[0], py - proj[1])
                if d < bd and d <= maxd:
                    bd = d; best = (ln, i, A, B)
        return best

    def _draw_light_preview(self, idx):
        # draw polygon + small circle at node
        light = self.lights[idx]
        pts = light['area']['points']
        # canvas polygon expects flat coords list
        flat = []
        for p in pts:
            flat += [p[0], p[1]]
        poly = self.canvas.create_polygon(flat, fill='', outline='purple', width=2, tags=(f'light_preview_{light["id"]}',))
        node_pos = self._node_pos(light['node'])
        circ = self.canvas.create_oval(node_pos[0]-6, node_pos[1]-6, node_pos[0]+6, node_pos[1]+6, fill='purple')
        # store tags not necessary; when saving map we will regenerate drawing on load

    def _find_light_near(self, x, y, rad=18):
        for i, L in enumerate(self.lights):
            nx, ny = self._node_pos(L['node'])
            if math.hypot(nx - x, ny - y) < rad:
                return i
        return None

    def rotate_selected_light(self):
        if self.selected_light_index is None:
            # if none selected ask to pick one
            idx = simpledialog.askinteger('Rotate Light', 'Enter lights list index (1..):', parent=self.master, minvalue=1, initialvalue=1)
            if idx is None: return
            idx -= 1
            if idx < 0 or idx >= len(self.lights):
                messagebox.showerror('Error', 'Invalid index')
                return
            self.selected_light_index = idx
        L = self.lights[self.selected_light_index]
        rot = (L.get('area',{}).get('rot',0) + 90) % 360
        # rotate polygon around node position
        node_pos = self._node_pos(L['node'])
        pts = L['area']['points']
        new_pts = []
        ang = math.radians(90)
        ca = math.cos(ang); sa = math.sin(ang)
        for (px,py) in pts:
            rx = px - node_pos[0]; ry = py - node_pos[1]
            nx = rx * ca - ry * sa
            ny = rx * sa + ry * ca
            new_pts.append([node_pos[0] + nx, node_pos[1] + ny])
        L['area']['points'] = new_pts
        L['area']['rot'] = rot
        # redraw canvas: simplest is to clear previews and redraw lanes/hubs/joins then previews
        self.redraw_canvas()

    def redraw_canvas(self):
        self.canvas.delete('all')
        # draw roads
        for r in self.roads:
            try:
                self.canvas.create_rectangle(r['x'], r['y'], r['x']+r['w'], r['y']+r['h'], fill='black')
            except Exception:
                pass
        # draw lanes (centerlines)
        for ln in self.lanes:
            pts = ln.get('points', [])
            if len(pts) >= 2:
                for i in range(len(pts)-1):
                    p1 = pts[i]; p2 = pts[i+1]
                    self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill='gray', width=6)
        # draws joins
        for j in self.joins:
            self.canvas.create_rectangle(j['x']-6, j['y']-6, j['x']+6, j['y']+6, fill='orange')
        # hubs + in/out nodes (from self.nodes)
        for h in self.hubs:
            self.canvas.create_oval(h['x']-12, h['y']-12, h['x']+12, h['y']+12, fill='gold')
            self.canvas.create_text(h['x'], h['y']-18, text=h.get('name',''))
        # raw nodes (including hub in/out)
        for n in self.nodes:
            t = n.get('type','')
            if t == 'hub_in':
                self.canvas.create_oval(n['x']-6, n['y']-6, n['x']+6, n['y']+6, fill='green')
            elif t == 'hub_out':
                self.canvas.create_oval(n['x']-6, n['y']-6, n['x']+6, n['y']+6, fill='red')
            else:
                self.canvas.create_oval(n['x']-4, n['y']-4, n['x']+4, n['y']+4, fill='black')
        # redraw lights
        for i in range(len(self.lights)):
            self._draw_light_preview(i)

    def finish_lane(self):
        if len(self.current_lane_points) < 2:
            messagebox.showinfo('Finish Lane', 'Need at least 2 points')
            return
        lid = self.new_id(); pts = list(self.current_lane_points)
        # auto determine direction by point sequence
        self.lanes.append({'id':lid, 'points':pts, 'dir':'both'})
        self.current_lane_points = []
        messagebox.showinfo('Lane', 'Lane saved')
        self.redraw_canvas()

    def save(self):
        filepath = filedialog.asksaveasfilename(defaultextension='.json', filetypes=[('JSON','*.json')])
        if not filepath: return
        # collect nodes: include self.nodes (hub in/out and any manual nodes)
        data = {'roads':self.roads, 'hubs':self.hubs, 'joins':self.joins, 'lanes':self.lanes, 'nodes':self.nodes, 'lights':self.lights, 'symbols':[]}
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        messagebox.showinfo('Saved', 'Map saved')

    def load(self):
        fp = filedialog.askopenfilename(filetypes=[('JSON','*.json')])
        if not fp: return
        with open(fp, 'r', encoding='utf-8') as f:
            data = json.load(f)
        self.roads = data.get('roads', []); self.hubs = data.get('hubs', []); self.joins = data.get('joins', [])
        self.lanes = data.get('lanes', []); self.nodes = data.get('nodes', [])
        self.lights = data.get('lights', [])
        # ensure next_id higher than all ids present
        maxid = 0
        for coll in (self.roads, self.hubs, self.joins, self.lanes, self.nodes, self.lights):
            for item in coll:
                try:
                    if item.get('id',0) > maxid: maxid = item.get('id',0)
                except Exception:
                    pass
        self.next_id = maxid + 1
        self.redraw_canvas()

if __name__ == '__main__':
    root = tk.Tk()
    app = Editor(root)
    root.mainloop()
