from threading import Event
from collections import defaultdict
import time

class Auction:
    def __init__(self):
        self.bids = defaultdict(list)
        self.assignment = {}
        self.round_complete = Event()

    def submit_bid(self, agent_id: int, task_id: int, bid_value: float):
        bt = time.time()
        self.bids[task_id].append((agent_id, bid_value, bt))

    def run_auction_round(self, task_ids: list, timeout: float):
        time.sleep(timeout)
        for t in task_ids:
            bids_for_t = self.bids.get(t, [])
            if not bids_for_t:
                continue
            winner = min(bids_for_t, key=lambda x: x[1])
            self.assignment[t] = winner[0]
        self.round_complete.set()

    def get_assignment(self, task_id: int):
        return self.assignment.get(task_id, None)

    def reset_round(self):
        self.bids.clear()
        self.assignment.clear()
        if self.round_complete.is_set():
            self.round_complete.clear()
