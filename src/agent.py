import threading
import time
from random import uniform, choice
from src.config import (MIN_SPEED, MAX_SPEED, MIN_CAPACITY, MAX_CAPACITY,
                        AUCTION_INTERVAL, BID_TIMEOUT, ACCEPT_TIMEOUT)
from src.environment import Environment

class Agent(threading.Thread):
    def __init__(self, agent_id: int, env: Environment, auction, all_tasks: dict, sim_start_time: float):
        super().__init__()
        self.agent_id = agent_id
        self.env = env
        self.auction = auction
        self.all_tasks = all_tasks
        self.sim_start_time = sim_start_time

        self.position = self.random_start_position()
        self.speed = uniform(MIN_SPEED, MAX_SPEED)
        self.capacity = int(uniform(MIN_CAPACITY, MAX_CAPACITY))
        self.current_load = 0

        self.assigned_tasks = []
        self.plan = []
        self.current_route = []
        self.route_index = 0
        self.completed_tasks_count = 0
        self.locked_edges = {}
        self.stop_event = threading.Event()

    def random_start_position(self):
        verts = list(self.env.graph.vertices())
        return choice(verts) if verts else 0

    def run(self):
        next_auction_time = time.time()
        while not self.stop_event.is_set():
            now_real = time.time()
            now_sim = now_real - self.sim_start_time

            if now_real >= next_auction_time:
                self.execute_auction()
                next_auction_time = now_real + AUCTION_INTERVAL

            if self.plan:
                self.follow_route(now_real)

            self.monitor_plan(now_real)
            time.sleep(0.1)

    def execute_auction(self):
        free_tasks = [t_id for t_id, info in self.all_tasks.items()
                      if info.get('assigned_to') is None]
        for t_id in free_tasks:
            cost = self.estimate_cost(t_id)
            if cost < float('inf'):
                self.auction.submit_bid(self.agent_id, t_id, cost)
        time.sleep(BID_TIMEOUT)
        for t_id in free_tasks:
            winner = self.auction.get_assignment(t_id)
            if winner == self.agent_id:
                self.all_tasks[t_id]['assigned_to'] = self.agent_id
                self.assigned_tasks.append(t_id)
        time.sleep(ACCEPT_TIMEOUT)
        self.plan_tasks()

    def estimate_cost(self, task_id: int) -> float:
        task = self.all_tasks[task_id]
        pickup = task['pickup_node']
        delivery = task['delivery_node']
        e_j = task['e']
        l_j = task['l']
        w_j = task['w']

        if w_j + self.current_load > self.capacity:
            return float('inf')

        now_sim = time.time() - self.sim_start_time

        path_to_pick, dist1 = self.env.graph.shortest_path(self.position, pickup)
        if path_to_pick is None:
            return float('inf')

        path_to_deliv, dist2 = self.env.graph.shortest_path(pickup, delivery)
        if path_to_deliv is None:
            return float('inf')

        total_time = (dist1 + dist2) / self.speed
        if now_sim + total_time > l_j:
            return float('inf')
        return total_time

    def plan_tasks(self):
        remaining = list(self.assigned_tasks)
        current_pos = self.position
        new_plan = []
        while remaining:
            best_t = None
            best_cost = float('inf')
            for t_id in remaining:
                pickup = self.all_tasks[t_id]['pickup_node']
                delivery = self.all_tasks[t_id]['delivery_node']
                path1, d1 = self.env.graph.shortest_path(current_pos, pickup)
                if path1 is None:
                    continue
                _, d2 = self.env.graph.shortest_path(pickup, delivery)
                cost = (d1 + d2) / self.speed
                if cost < best_cost:
                    best_cost = cost
                    best_t = t_id
            if best_t is None:
                break
            new_plan.append(best_t)
            remaining.remove(best_t)
            current_pos = self.all_tasks[best_t]['delivery_node']
        self.plan = new_plan

    def follow_route(self, now_real: float):
        now_sim = time.time() - self.sim_start_time
        if self.route_index >= len(self.current_route):
            if not self.plan:
                return
            next_task = self.plan[0]
            pickup = self.all_tasks[next_task]['pickup_node']
            delivery = self.all_tasks[next_task]['delivery_node']
            path1, dist1 = self.env.shortest_path(self.position, pickup, now_sim)
            if path1 is None:
                self.handle_reroute()
                return
            eta1 = now_sim + dist1 / self.speed
            path2, dist2 = self.env.shortest_path(pickup, delivery, eta1)
            if path2 is None:
                self.handle_reroute()
                return
            combined = path1 + path2[1:]
            self.current_route = combined
            self.route_index = 0

        now_sim = time.time() - self.sim_start_time
        u = self.current_route[self.route_index]
        if self.route_index + 1 < len(self.current_route):
            v = self.current_route[self.route_index + 1]
        else:
            self.finish_current_task(now_sim)
            return

        weight_uv = self.get_edge_weight(u, v)
        travel_time = weight_uv / self.speed
        self.locked_edges[(u, v)] = (now_sim, now_sim + travel_time)
        time.sleep(travel_time)
        self.position = v
        self.route_index += 1

    def get_edge_weight(self, u: int, v: int) -> float:
        for nbr, w in self.env.graph.neighbors(u):
            if nbr == v:
                return w
        return float('inf')

    def finish_current_task(self, now_sim: float):
        if not self.plan:
            return
        finished_task = self.plan.pop(0)
        self.assigned_tasks.remove(finished_task)
        w_j = self.all_tasks[finished_task]['w']
        self.current_load -= w_j
        self.all_tasks[finished_task]['completed'] = True
        self.completed_tasks_count += 1
        print(f"[{now_sim:.1f}s] Agent {self.agent_id} ДОСТАВИЛ задачу {finished_task}")
        self.cleanup_locks()

    def cleanup_locks(self):
        now_sim = time.time() - self.sim_start_time
        to_remove = []
        for (u, v), (t_start, t_end) in self.locked_edges.items():
            if t_end <= now_sim:
                to_remove.append((u, v))
        for edge in to_remove:
            del self.locked_edges[edge]

    def monitor_plan(self, now_real: float):
        if not self.plan:
            return
        est_pos = self.position
        est_time_sim = time.time() - self.sim_start_time
        for t_id in self.plan:
            pickup = self.all_tasks[t_id]['pickup_node']
            delivery = self.all_tasks[t_id]['delivery_node']
            e_j = self.all_tasks[t_id]['e']
            l_j = self.all_tasks[t_id]['l']
            path1, d1 = self.env.graph.shortest_path(est_pos, pickup)
            if path1 is None:
                self.handle_reroute()
                return
            eta1 = est_time_sim + d1 / self.speed
            if eta1 > l_j:
                self.handle_reroute()
                return
            path2, d2 = self.env.graph.shortest_path(pickup, delivery)
            if path2 is None:
                self.handle_reroute()
                return
            eta2 = eta1 + d2 / self.speed
            if eta2 > l_j:
                self.handle_reroute()
                return
            est_pos = delivery
            est_time_sim = eta2

    def handle_reroute(self):
        for t_id in list(self.plan):
            self.all_tasks[t_id]['assigned_to'] = None
            self.plan.remove(t_id)
            self.assigned_tasks.remove(t_id)

    def stop(self):
        self.stop_event.set()
