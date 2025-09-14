# warehouse_gui_sim.py
# GUI + Simulation for VLM (simple lift) vs VRM (combined movers: horizontal + vertical)
# No external libraries. Run: python warehouse_gui_sim.py

import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass
import random
import heapq
import statistics
from typing import List, Optional, Tuple

# =========================
# Discrete-Event Simulation
# =========================
class EventLoop:
    def __init__(self, until_time: float):
        self.time = 0.0
        self.until_time = until_time
        self._q = []
        self._counter = 0
    def schedule(self, when: float, fn):
        self._counter += 1
        heapq.heappush(self._q, (when, self._counter, fn))
    def run(self):
        while self._q and self.time <= self.until_time:
            when, _, fn = heapq.heappop(self._q)
            if when > self.until_time:
                break
            self.time = when
            fn(self)

class Resource:
    """Single-capacity resource with FCFS queue (e.g., VLM lift)."""
    def __init__(self, name: str, capacity: int = 1):
        self.name = name
        self.capacity = capacity
        self.busy = 0
        self.queue: List[Tuple[float, dict]] = []
        self.busy_time = 0.0
        self._last_change = 0.0
    def _update_busy_time(self, now: float):
        if self.busy > 0:
            self.busy_time += (now - self._last_change)
        self._last_change = now
    def request(self, now: float, job: dict, start_callback):
        self._update_busy_time(now)
        if self.busy < self.capacity:
            self.busy += 1
            start_callback(now)
        else:
            self.queue.append((now, job))
    def release_and_maybe_start_next(self, now: float, start_callback):
        self._update_busy_time(now)
        self.busy -= 1
        if self.queue and self.busy < self.capacity:
            _, job = self.queue.pop(0)
            self.busy += 1
            start_callback(now, job_override=job)
    def utilization(self, sim_time: float) -> float:
        return (self.busy_time / sim_time) if sim_time > 0 else 0.0

class AgentPool:
    """
    Pool of N combined movers (VRM).
    Tracks each agent's busy state, busy time, and current horizontal position.
    FCFS queue; when an agent frees, it starts the next job immediately.
    """
    def __init__(self, n_agents: int, init_pos: float = 0.0):
        self.n = n_agents
        self.positions = [init_pos for _ in range(n_agents)]
        self.busy = [False] * n_agents
        self.busy_time = [0.0] * n_agents
        self.last_change = [0.0] * n_agents
        self.queue: List[dict] = []
    def _start_agent(self, idx: int, now: float, job: dict, start_callback):
        self.busy[idx] = True
        self.last_change[idx] = now
        start_callback(now, agent_idx=idx, job_override=job)
    def request(self, now: float, job: dict, start_callback, prefer_x: float):
        free_indices = [i for i in range(self.n) if not self.busy[i]]
        if free_indices:
            idx = min(free_indices, key=lambda i: abs(self.positions[i] - prefer_x))
            self._start_agent(idx, now, job, start_callback)
        else:
            self.queue.append(job)
    def release(self, idx: int, now: float, new_position: float, start_callback=None):
        self.busy_time[idx] += now - self.last_change[idx]
        self.busy[idx] = False
        self.positions[idx] = new_position
        if self.queue:
            job = self.queue.pop(0)
            self._start_agent(idx, now, job, start_callback)
    def utilization(self, sim_time: float) -> float:
        total_busy = sum(self.busy_time)
        return (total_busy / (self.n * sim_time)) if sim_time > 0 and self.n > 0 else 0.0

# =========================
# Parameters
# =========================
@dataclass
class Common:
    sim_hours: float = 2.0          # total simulated time (hours)
    arrival_rate_per_h: float = 400 # average incoming orders per hour
    seed: int = 42

@dataclass
class VLMParams:
    levels_per_side: int = 20
    level_height_m: float = 0.3
    access_height_m: float = 3.0
    lift_speed_mps: float = 1.2
    tray_load_unload_s: float = 2.0
    pick_time_s_mean: float = 12.0  # blocks the lift during pick

@dataclass
class VRMCombinedParams:
    n_racks: int = 6
    levels_per_rack: int = 20
    level_height_m: float = 0.3
    rack_spacing_m: float = 3.0
    n_stations: int = 3
    n_agents: int = 4
    horiz_speed_mps: float = 2.0
    vertical_speed_mps: float = 1.0
    load_unload_vertical_s: float = 2.0
    load_unload_horizontal_s: float = 2.0
    block_during_pick: bool = False
    pick_time_s_mean: float = 10.0

# =========================
# VLM Simulator (simple lift)
# =========================
class VLMSim:
    def __init__(self, common: Common, p: VLMParams):
        self.c = common
        self.p = p
        self.sim_time_s = self.c.sim_hours * 3600.0
        self.ev = EventLoop(until_time=self.sim_time_s)
        self.lift = Resource("vlm_lift", capacity=1)

        self.completed = 0
        self.arrivals = 0
        self.cycle_times: List[float] = []
        self.wait_times: List[float] = []

    def _exp(self, mean: float) -> float:
        return random.expovariate(1.0 / mean) if mean > 0 else 0.0

    def _interarrival_s(self) -> float:
        lam = self.c.arrival_rate_per_h / 3600.0
        return random.expovariate(lam) if lam > 0 else float('inf')

    def _tray_height_sample(self) -> float:
        H = self.p.levels_per_side * self.p.level_height_m
        return random.uniform(0.0, H)

    def _service_time(self, tray_h: float) -> float:
        vertical = 2.0 * abs(self.p.access_height_m - tray_h) / self.p.lift_speed_mps
        handling = 2.0 * self.p.tray_load_unload_s + self._exp(self.p.pick_time_s_mean)
        return vertical + handling

    def start(self):
        random.seed(self.c.seed)
        t = self._interarrival_s()
        self.ev.schedule(t, lambda ev: self._arrival(ev, arrival_time=t))
        self.ev.run()

    def _arrival(self, ev: EventLoop, arrival_time: float):
        if arrival_time > self.sim_time_s:
            return
        self.arrivals += 1
        job = {"arrival": arrival_time, "tray_h": self._tray_height_sample()}

        def start_service(now: float, job_override: Optional[dict] = None):
            j = job if job_override is None else job_override
            st = self._service_time(j["tray_h"])
            self.wait_times.append(now - j["arrival"])
            ev.schedule(now + st, lambda ev2: self._finish(ev2, start_time=now, job=j))

        self.lift.request(arrival_time, job, start_service)

        next_arrival = arrival_time + self._interarrival_s()
        self.ev.schedule(next_arrival, lambda ev2: self._arrival(ev2, arrival_time=next_arrival))

    def _finish(self, ev: EventLoop, start_time: float, job: dict):
        self.cycle_times.append(ev.time - job["arrival"])
        self.completed += 1

        def start_next(now: float, job_override: Optional[dict] = None):
            j = job_override
            st = self._service_time(j["tray_h"])
            self.wait_times.append(now - j["arrival"])
            ev.schedule(now + st, lambda ev2: self._finish(ev2, start_time=now, job=j))

        self.lift.release_and_maybe_start_next(ev.time, start_next)

    def results(self):
        sim_h = self.c.sim_hours
        th_put = self.completed / sim_h if sim_h > 0 else 0.0
        avg_cycle = statistics.mean(self.cycle_times) if self.cycle_times else 0.0
        avg_wait = statistics.mean(self.wait_times) if self.wait_times else 0.0
        util = self.lift.utilization(self.sim_time_s)
        return {
            "throughput": th_put,
            "avg_cycle_time_s": avg_cycle,
            "avg_wait_time_s": avg_wait,
            "utilization": util,
            "arrivals": self.arrivals,
            "completed": self.completed,
            "theoretical": self.theoretical_capacity_jobs_per_h()
        }

    def theoretical_capacity_jobs_per_h(self) -> float:
        H = self.p.levels_per_side * self.p.level_height_m
        a = self.p.access_height_m
        # E|X-a| for X~Uniform(0,H)
        e_abs = (a*a + (H - a)*(H - a)) / (2.0 * H) if H > 0 else 0.0
        vertical = 2.0 * e_abs / self.p.lift_speed_mps
        handling = 2.0 * self.p.tray_load_unload_s + self.p.pick_time_s_mean
        avg_service = vertical + handling
        return 3600.0 / avg_service if avg_service > 0 else 0.0

# =========================
# VRM Simulator (Combined movers)
# =========================
class VRMSimCombined:
    """
    Each job:
      1) Assign a free combined mover (nearest to rack).
      2) Horizontal: agent_x -> rack_x.
      3) Vertical round-trip to level.
      4) Horizontal: rack_x -> station_x.
      5) Handling at rack & station; optional pick blocking.
      6) Mover ends at station position.
    """
    def __init__(self, common: Common, p: VRMCombinedParams):
        self.c = common
        self.p = p
        self.sim_time_s = self.c.sim_hours * 3600.0
        self.ev = EventLoop(until_time=self.sim_time_s)

        # Layout positions (meters along X)
        self.rack_positions = [i * self.p.rack_spacing_m for i in range(self.p.n_racks)]
        if self.p.n_stations <= 1:
            mid = 0.5 * (self.p.n_racks - 1) * self.p.rack_spacing_m
            self.station_positions = [mid]
        else:
            span = (self.p.n_racks - 1) * self.p.rack_spacing_m
            step = span / (self.p.n_stations - 1)
            self.station_positions = [i * step for i in range(self.p.n_stations)]

        # Agent pool starts at middle station
        start_x = self.station_positions[len(self.station_positions)//2]
        self.agents = AgentPool(n_agents=self.p.n_agents, init_pos=start_x)

        # Stats
        self.completed = 0
        self.arrivals = 0
        self.cycle_times: List[float] = []
        self.wait_times: List[float] = []
        self.horiz_times: List[float] = []
        self.vert_times: List[float] = []
        self.handle_times: List[float] = []

    def _exp(self, mean: float) -> float:
        return random.expovariate(1.0 / mean) if mean > 0 else 0.0

    def _interarrival_s(self) -> float:
        lam = self.c.arrival_rate_per_h / 3600.0
        return random.expovariate(lam) if lam > 0 else float('inf')

    def _sample_rack(self) -> int:
        return random.randrange(self.p.n_racks)

    def _sample_level(self) -> int:
        return random.randint(1, self.p.levels_per_rack)

    def _station_index(self) -> int:
        return random.randrange(self.p.n_stations)

    def _vertical_time(self, level: int) -> float:
        height = level * self.p.level_height_m
        return 2.0 * height / self.p.vertical_speed_mps

    def _horiz_time(self, x1: float, x2: float) -> float:
        return abs(x2 - x1) / self.p.horiz_speed_mps

    def start(self):
        random.seed(self.c.seed)
        t = self._interarrival_s()
        self.ev.schedule(t, lambda ev: self._arrival(ev, arrival_time=t))
        self.ev.run()

    def _arrival(self, ev: EventLoop, arrival_time: float):
        if arrival_time > self.sim_time_s:
            return
        self.arrivals += 1
        rack = self._sample_rack()
        level = self._sample_level()
        station = self._station_index()
        x_rack = self.rack_positions[rack]
        x_station = self.station_positions[station]

        job = {
            "arrival": arrival_time,
            "rack": rack,
            "level": level,
            "station": station,
            "_x_rack": x_rack,
            "_x_station": x_station
        }

        def start_service(now: float, agent_idx: int = -1, job_override: Optional[dict] = None):
            j = job if job_override is None else job_override
            # Horizontal reposition to rack from agent's current position
            x_agent = self.agents.positions[agent_idx]
            h1 = self._horiz_time(x_agent, j["_x_rack"])
            # Vertical roundtrip
            v = self._vertical_time(j["level"])
            # Handling at rack
            hdl_rack = self.p.load_unload_vertical_s
            # Horizontal rack -> station
            h2 = self._horiz_time(j["_x_rack"], j["_x_station"])
            # Handling at station (+ optional pick blocking)
            pick = self._exp(self.p.pick_time_s_mean) if self.p.block_during_pick else 0.0
            hdl_station = self.p.load_unload_horizontal_s + pick

            st = h1 + v + hdl_rack + h2 + hdl_station

            # record waits and components
            self.wait_times.append(now - j["arrival"])
            j["_h1"] = h1
            j["_v"] = v
            j["_hdl"] = hdl_rack + hdl_station
            j["_agent_idx"] = agent_idx

            ev.schedule(now + st, lambda ev2: self._finish(ev2, start_time=now, job=j))

        # Request an agent, prefer the nearest to rack x
        self.agents.request(arrival_time, job, start_service, prefer_x=x_rack)

        # Next arrival
        next_arrival = arrival_time + self._interarrival_s()
        self.ev.schedule(next_arrival, lambda ev2: self._arrival(ev2, arrival_time=next_arrival))

    def _finish(self, ev: EventLoop, start_time: float, job: dict):
        self.completed += 1
        self.cycle_times.append(ev.time - job["arrival"])
        self.horiz_times.append(job["_h1"] + self._horiz_time(job["_x_rack"], job["_x_station"]))
        self.vert_times.append(job["_v"])
        self.handle_times.append(job["_hdl"])

        # Release agent at station position (agent ends at station)
        agent_idx = job["_agent_idx"]

        def start_next(now: float, agent_idx: int = agent_idx, job_override: Optional[dict] = None):
            j = job_override
            x_agent_now = self.agents.positions[agent_idx]
            h1 = self._horiz_time(x_agent_now, j["_x_rack"])
            v = self._vertical_time(j["level"])
            hdl_rack = self.p.load_unload_vertical_s
            h2 = self._horiz_time(j["_x_rack"], j["_x_station"])
            pick = self._exp(self.p.pick_time_s_mean) if self.p.block_during_pick else 0.0
            hdl_station = self.p.load_unload_horizontal_s + pick
            st = h1 + v + hdl_rack + h2 + hdl_station

            self.wait_times.append(now - j["arrival"])
            j["_h1"] = h1
            j["_v"] = v
            j["_hdl"] = hdl_rack + hdl_station
            j["_agent_idx"] = agent_idx

            ev.schedule(now + st, lambda ev2: self._finish(ev2, start_time=now, job=j))

        self.agents.release(agent_idx, ev.time, new_position=job["_x_station"], start_callback=start_next)

    def results(self):
        sim_h = self.c.sim_hours
        th_put = self.completed / sim_h if sim_h > 0 else 0.0
        avg_cycle = statistics.mean(self.cycle_times) if self.cycle_times else 0.0
        avg_wait = statistics.mean(self.wait_times) if self.wait_times else 0.0
        util = self.agents.utilization(self.sim_time_s)
        avg_horiz = statistics.mean(self.horiz_times) if self.horiz_times else 0.0
        avg_vert = statistics.mean(self.vert_times) if self.vert_times else 0.0
        avg_hdl = statistics.mean(self.handle_times) if self.handle_times else 0.0
        return {
            "throughput": th_put,
            "avg_cycle_time_s": avg_cycle,
            "avg_wait_time_s": avg_wait,
            "utilization": util,
            "arrivals": self.arrivals,
            "completed": self.completed,
            "avg_horizontal_time_s": avg_horiz,
            "avg_vertical_time_s": avg_vert,
            "avg_handling_time_s": avg_hdl,
            "theoretical": self.theoretical_capacity_jobs_per_h(),
            "agent_positions": list(self.agents.positions), # for drawing
            "rack_positions": list(self.rack_positions),
            "station_positions": list(self.station_positions)
        }

    def theoretical_capacity_jobs_per_h(self) -> float:
        # Expected vertical time:
        E_height = self.p.level_height_m * (self.p.levels_per_rack + 1) / 2.0
        E_vert = 2.0 * E_height / self.p.vertical_speed_mps

        # Expected horizontal distances between a random station and a random rack
        racks = self.rack_positions
        stats = self.station_positions
        total = 0.0
        for xr in racks:
            for xs in stats:
                total += abs(xs - xr)
        E_sr = total / (len(racks) * len(stats)) if racks and stats else 0.0
        E_horiz = (2.0 * E_sr) / self.p.horiz_speed_mps

        # Handling
        E_handle = self.p.load_unload_vertical_s + self.p.load_unload_horizontal_s + \
                   (self.p.pick_time_s_mean if self.p.block_during_pick else 0.0)

        E_service = E_vert + E_horiz + E_handle
        cap_per_agent = 3600.0 / E_service if E_service > 0 else 0.0
        return self.p.n_agents * cap_per_agent

# =========================
# Tkinter GUI
# =========================
class WarehouseGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("VLM vs VRM Simulator (with Visualization)")

        # Main layout: Left controls, right results + canvas
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # ---- Left: Parameters panel ----
        left = ttk.Frame(self, padding=10)
        left.grid(row=0, column=0, sticky="ns")

        # COMMON
        ttk.Label(left, text="Common (Workload)", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, sticky="w", pady=(0,5))
        self.common_vars = {
            "sim_hours": self._entry(left, "Sim hours", 1, default="2.0"),
            "arrival_rate_per_h": self._entry(left, "Arrival rate (jobs/h)", 2, default="400"),
            "seed": self._entry(left, "Random seed", 3, default="1"),
        }

        # VLM
        ttk.Label(left, text="VLM (Simple Lift)", font=("Segoe UI", 10, "bold")).grid(row=4, column=0, sticky="w", pady=(10,5))
        self.vlm_vars = {
            "levels_per_side": self._entry(left, "Levels per side", 5, default="20"),
            "level_height_m": self._entry(left, "Level height (m)", 6, default="0.3"),
            "access_height_m": self._entry(left, "Access height (m)", 7, default="3.0"),
            "lift_speed_mps": self._entry(left, "Lift speed (m/s)", 8, default="1.2"),
            "tray_load_unload_s": self._entry(left, "Tray load/unload (s)", 9, default="2.0"),
            "pick_time_s_mean": self._entry(left, "Pick time mean (s)", 10, default="12.0"),
        }

        # VRM
        ttk.Label(left, text="VRM (Combined Movers)", font=("Segoe UI", 10, "bold")).grid(row=11, column=0, sticky="w", pady=(10,5))
        self.vrm_vars = {
            "n_racks": self._entry(left, "Racks (aisles)", 12, default="6"),
            "levels_per_rack": self._entry(left, "Levels per rack", 13, default="20"),
            "level_height_m": self._entry(left, "Level height (m)", 14, default="0.3"),
            "rack_spacing_m": self._entry(left, "Rack spacing (m)", 15, default="3.0"),
            "n_stations": self._entry(left, "Stations", 16, default="3"),
            "n_agents": self._entry(left, "Movers (agents)", 17, default="4"),
            "horiz_speed_mps": self._entry(left, "Horizontal speed (m/s)", 18, default="2.0"),
            "vertical_speed_mps": self._entry(left, "Vertical speed (m/s)", 19, default="1.0"),
            "load_unload_vertical_s": self._entry(left, "Rack handling (s)", 20, default="2.0"),
            "load_unload_horizontal_s": self._entry(left, "Station handling (s)", 21, default="2.0"),
            "pick_time_s_mean": self._entry(left, "Pick time mean (s)", 22, default="10.0"),
        }
        self.vrm_block_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="Block movers during pick (VRM)", variable=self.vrm_block_var).grid(row=23, column=0, sticky="w", pady=(5,10))

        # Run button
        run_btn = ttk.Button(left, text="Run Simulation", command=self.run_sim)
        run_btn.grid(row=24, column=0, sticky="ew", pady=(5,15))

        # ---- Right: Results + Canvas ----
        right = ttk.Frame(self, padding=10)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        # Results grid
        results = ttk.Frame(right)
        results.grid(row=0, column=0, sticky="ew")
        for i in range(4): results.columnconfigure(i, weight=1)

        # Headings
        ttk.Label(results, text="VLM Results", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(results, text="VRM Results", font=("Segoe UI", 10, "bold")).grid(row=0, column=2, sticky="w")

        # VLM result labels
        self.lbl_vlm = self._make_result_block(results, start_row=1, start_col=0)
        # VRM result labels
        self.lbl_vrm = self._make_result_block(results, start_row=1, start_col=2, vrm=True)

        # Canvas for drawing
        self.canvas = tk.Canvas(right, bg="white", height=520)
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=(10,0))

        # First run on startup
        self.after(100, self.run_sim)

    # --- Helpers to build UI ---
    def _entry(self, parent, label, row, default=""):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        var = tk.StringVar(value=default)
        ent = ttk.Entry(parent, textvariable=var, width=20)
        ent.grid(row=row, column=1, sticky="w")
        return var

    def _make_result_block(self, parent, start_row=1, start_col=0, vrm=False):
        labels = {}
        items = [
            ("Throughput (jobs/h)", "throughput"),
            ("Avg cycle (s)", "avg_cycle_time_s"),
            ("Avg wait (s)", "avg_wait_time_s"),
            ("Utilization (%)", "utilization"),
            ("Arrivals", "arrivals"),
            ("Completed", "completed"),
            ("Theoretical (jobs/h)", "theoretical"),
        ]
        if vrm:
            items.extend([
                ("Avg horiz (s)", "avg_horizontal_time_s"),
                ("Avg vert (s)", "avg_vertical_time_s"),
                ("Avg handling (s)", "avg_handling_time_s"),
            ])
        r = start_row
        for text, key in items:
            ttk.Label(parent, text=text + ":").grid(row=r, column=start_col, sticky="w")
            val = ttk.Label(parent, text="-")
            val.grid(row=r, column=start_col+1, sticky="w")
            labels[key] = val
            r += 1
        return labels

    # --- Run simulation & update UI ---
    def run_sim(self):
        try:
            # Read parameters
            c = Common(
                sim_hours=float(self.common_vars["sim_hours"].get()),
                arrival_rate_per_h=float(self.common_vars["arrival_rate_per_h"].get()),
                seed=int(float(self.common_vars["seed"].get())),
            )
            vlm_p = VLMParams(
                levels_per_side=int(float(self.vlm_vars["levels_per_side"].get())),
                level_height_m=float(self.vlm_vars["level_height_m"].get()),
                access_height_m=float(self.vlm_vars["access_height_m"].get()),
                lift_speed_mps=float(self.vlm_vars["lift_speed_mps"].get()),
                tray_load_unload_s=float(self.vlm_vars["tray_load_unload_s"].get()),
                pick_time_s_mean=float(self.vlm_vars["pick_time_s_mean"].get()),
            )
            vrm_p = VRMCombinedParams(
                n_racks=int(float(self.vrm_vars["n_racks"].get())),
                levels_per_rack=int(float(self.vrm_vars["levels_per_rack"].get())),
                level_height_m=float(self.vrm_vars["level_height_m"].get()),
                rack_spacing_m=float(self.vrm_vars["rack_spacing_m"].get()),
                n_stations=int(float(self.vrm_vars["n_stations"].get())),
                n_agents=int(float(self.vrm_vars["n_agents"].get())),
                horiz_speed_mps=float(self.vrm_vars["horiz_speed_mps"].get()),
                vertical_speed_mps=float(self.vrm_vars["vertical_speed_mps"].get()),
                load_unload_vertical_s=float(self.vrm_vars["load_unload_vertical_s"].get()),
                load_unload_horizontal_s=float(self.vrm_vars["load_unload_horizontal_s"].get()),
                block_during_pick=self.vrm_block_var.get(),
                pick_time_s_mean=float(self.vrm_vars["pick_time_s_mean"].get()),
            )

            # Run sims
            vlm = VLMSim(c, vlm_p); vlm.start(); res_vlm = vlm.results()
            vrm = VRMSimCombined(c, vrm_p); vrm.start(); res_vrm = vrm.results()

            # Update result labels
            self._update_results(self.lbl_vlm, res_vlm)
            self._update_results(self.lbl_vrm, res_vrm)

            # Draw visualization
            self._draw_visual(vlm_p, vrm_p, res_vrm)

        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _update_results(self, label_map, res: dict):
        def fmt(x):
            return f"{x:.2f}" if isinstance(x, float) else str(x)
        # Map and special-case utilization to %
        for k, lbl in label_map.items():
            if k not in res: continue
            v = res[k]
            if k == "utilization":
                lbl.config(text=f"{v*100:.2f}")
            else:
                lbl.config(text=fmt(v))

    # --- Drawing ---
    def _draw_visual(self, vlm_p: VLMParams, vrm_p: VRMCombinedParams, res_vrm: dict):
        self.canvas.delete("all")
        W = self.canvas.winfo_width() or 1000
        H = 520

        # Compute span (meters) for VRM so we scale nicely
        span_m = max(1.0, (vrm_p.n_racks - 1) * vrm_p.rack_spacing_m)
        # Try to fit VRM into ~600 px width; also leave space for VLM left
        vrm_w_px = 620
        px_per_m = vrm_w_px / span_m

        # Offsets
        margin = 20
        vlm_area = (margin, margin, 300, H - margin)      # left block for VLM
        vrm_area = (320, margin, 320 + vrm_w_px, H - margin)  # right block for VRM

        # ---- Draw VLM (schematic) ----
        self._draw_vlm_block(vlm_area, vlm_p)

        # ---- Draw VRM (layout + movers) ----
        self._draw_vrm_block(vrm_area, vrm_p, res_vrm, px_per_m)

        # Titles
        self.canvas.create_text((vlm_area[0]+vlm_area[2])//2, 10, text="VLM", anchor="n", font=("Segoe UI", 10, "bold"))
        self.canvas.create_text((vrm_area[0]+vrm_area[2])//2, 10, text="VRM (Combined Movers)", anchor="n", font=("Segoe UI", 10, "bold"))

    def _draw_vlm_block(self, area, p: VLMParams):
        x1, y1, x2, y2 = area
        width = x2 - x1
        height = y2 - y1 - 40  # reduce for padding bottom

        # Total height of rack section (meters)
        Hm = p.levels_per_side * p.level_height_m
        # Scale vertical meters to pixels
        px_per_m_v = height / max(1.0, Hm)

        base_y = y1 + height

        # Draw two racks (left & right) with a gap in the middle for lift
        gap = 40
        rack_w = (width - gap - 40) // 2  # margins
        rx1 = x1 + 20
        rx2 = rx1 + rack_w
        rx3 = rx2 + gap
        rx4 = rx3 + rack_w

        # Left rack
        self.canvas.create_rectangle(rx1, base_y - Hm*px_per_m_v, rx2, base_y, outline="#4A90E2", width=2)
        # Right rack
        self.canvas.create_rectangle(rx3, base_y - Hm*px_per_m_v, rx4, base_y, outline="#4A90E2", width=2)

        # Lift (center line)
        lx = (rx2 + rx3) // 2
        self.canvas.create_line(lx, base_y - Hm*px_per_m_v, lx, base_y, fill="#E24A4A", width=3)

        # Access opening marker at access_height
        oy = base_y - p.access_height_m * px_per_m_v
        self.canvas.create_line(lx - 20, oy, lx + 20, oy, fill="#2ECC71", width=4)
        self.canvas.create_text(lx, oy - 12, text="Opening", fill="#2ECC71", font=("Segoe UI", 8))

        # Legend
        self.canvas.create_text((x1+x2)//2, y2-18, text="Racks (blue), Lift (red), Access (green)", fill="#666", font=("Segoe UI", 8))

    def _draw_vrm_block(self, area, p: VRMCombinedParams, res_vrm: dict, px_per_m: float):
        x1, y1, x2, y2 = area
        height = y2 - y1 - 40

        # Vertical scaling: show rack heights
        Hm = p.levels_per_rack * p.level_height_m
        px_per_m_v = height / max(1.0, Hm)
        base_y = y1 + height

        # Ground line
        self.canvas.create_line(x1, base_y, x2, base_y, fill="#999")

        # Racks (vertical rectangles)
        rack_px_positions = [x1 + xr * px_per_m for xr in res_vrm["rack_positions"]]
        rack_w = 10  # px width
        for rx in rack_px_positions:
            self.canvas.create_rectangle(rx - rack_w//2, base_y - Hm*px_per_m_v, rx + rack_w//2, base_y,
                                         outline="#4A90E2", width=2)

        # Stations (small squares at base)
        station_px_positions = [x1 + xs * px_per_m for xs in res_vrm["station_positions"]]
        for sx in station_px_positions:
            self.canvas.create_rectangle(sx - 6, base_y + 2, sx + 6, base_y + 14, fill="#2ECC71", outline="")

        # Agents (movers) as colored circles at their current positions (end of sim)
        colors = ["#E24A4A", "#F5A623", "#7B7FE0", "#50E3C2", "#B8E986", "#BD10E0", "#4A90E2", "#F8E71C"]
        for i, ax in enumerate(res_vrm["agent_positions"]):
            px = x1 + ax * px_per_m
            self.canvas.create_oval(px - 7, base_y - 14, px + 7, base_y, fill=colors[i % len(colors)], outline="")
            self.canvas.create_text(px, base_y - 18, text=f"A{i}", font=("Segoe UI", 8))

        # Legend
        self.canvas.create_text((x1+x2)//2, y2-18,
                                text="Racks (blue), Stations (green), Agents (colored)",
                                fill="#666", font=("Segoe UI", 8))

# =========================
# Main
# =========================
if __name__ == "__main__":
    app = WarehouseGUI()
    app.geometry("1100x650")
    app.mainloop()
