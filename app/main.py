import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, os.pardir))
sys.path.insert(0, PROJECT_ROOT)

from src.simulation import Simulation

def main():
    sim = Simulation()
    sim.run()

if __name__ == "__main__":
    main()
