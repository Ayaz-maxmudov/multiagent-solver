import threading
import time
from random import randint, uniform
from src.config import (NUM_AGENTS, INITIAL_ORDERS, SIMULATION_END_TIME,
                        AUCTION_INTERVAL, MAX_TIME_WINDOW, MIN_TIME_WINDOW,
                        MAX_CAPACITY)
from src.environment import Environment
from src.auction import Auction
from src.agent import Agent

class Simulation:
    def __init__(self):
        self.env = Environment()
        self.auction = Auction()
        self.all_tasks = {}
        self._generate_initial_tasks()
        self.start_time = time.time()
        self.agents = []
        for i in range(NUM_AGENTS):
            agent = Agent(i, self.env, self.auction, self.all_tasks, self.start_time)
            self.agents.append(agent)
        self.blocking_thread = threading.Thread(target=self.block_roads, daemon=True)

    def _generate_initial_tasks(self):
        task_id = 0
        verts = list(self.env.graph.vertices())
        for _ in range(INITIAL_ORDERS):
            if not verts:
                break
            pickup = randint(0, len(verts) - 1)
            delivery = randint(0, len(verts) - 1)
            while delivery == pickup:
                delivery = randint(0, len(verts) - 1)
            e = uniform(0, MAX_TIME_WINDOW - MIN_TIME_WINDOW)
            l = e + uniform(MIN_TIME_WINDOW, MAX_TIME_WINDOW - e)
            w = randint(1, MAX_CAPACITY)
            self.all_tasks[task_id] = {
                'pickup_node': verts[pickup],
                'delivery_node': verts[delivery],
                'e': e,
                'l': l,
                'w': w,
                'assigned_to': None,
                'completed': False
            }
            task_id += 1

    def block_roads(self):
        while True:
            time.sleep(uniform(15, 30))
            now_sim = time.time() - self.start_time
            self.env.random_block_edge(now_sim, duration=10.0)

    def run(self):
        self.blocking_thread.start()
        for agent in self.agents:
            agent.daemon = True
            agent.start()
        next_auction = time.time()
        while time.time() - self.start_time < SIMULATION_END_TIME:
            now = time.time()
            if now >= next_auction:
                free_tasks = [t_id for t_id, info in self.all_tasks.items()
                              if info['assigned_to'] is None and not info['completed']]
                if free_tasks:
                    self.auction.reset_round()
                    self.auction.run_auction_round(free_tasks, timeout=1.0)
                next_auction = now + AUCTION_INTERVAL
            time.sleep(0.5)
        for agent in self.agents:
            agent.stop()
        for agent in self.agents:
            agent.join(timeout=5)
        print("Симуляция завершена.")
        self.report()

    def report(self):
        print("=== Отчёт по выполнению заказов ===")
        completed_tasks = []
        unassigned = []
        in_progress = []
        for t_id, info in self.all_tasks.items():
            if info.get('completed', False):
                completed_tasks.append(t_id)
            elif info.get('assigned_to') is None:
                unassigned.append(t_id)
            else:
                in_progress.append(t_id)
        for agent in self.agents:
            print(f"Agent {agent.agent_id}: выполнил {agent.completed_tasks_count} заказов")
        total = len(self.all_tasks)
        print(f"Всего заказов: {total}")
        print(f"Выполнено (ID): {completed_tasks}")
        print(f"Невыполненных (ID): {unassigned}")
        print(f"В процессе (ID): {in_progress}")
