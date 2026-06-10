"""Universal gait generation — locomotion for any legged or aerial robot.

Generates foot placement trajectories and body motion for walking,
running, and flying. Works for any number of legs and any leg geometry.

Gait types:
- Walk:    3 feet on ground at all times (quadruped/hexapod)
- Trot:    diagonal pairs move together (quadruped)
- Crawl:   one leg at a time, slow & stable (quadruped)
- Gallop:  front pair then back pair, fast & unstable (quadruped)
- Bound:   both fronts then both rears (quadruped)
- Biped:   alternating legs with hip sway (biped)
- Tripod:  alternating groups of 3 legs (hexapod)
- CPG:     Central Pattern Generator — oscillator-based adaptive gait
- Flight:  multirotor attitude & motor control (quad/hex/octo)

Usage:
    from stacc.gait import TrotGait, CPGGenerator, FlightController

    gait = TrotGait(step_height=0.04, step_length=0.08, frequency=2.0)

    @robot.on_update
    def walk(dt):
        foot_targets = gait.compute(robot.time)
        for leg_id in range(4):
            angles = robot.leg_ik(leg_id, foot_targets[leg_id])
            robot.set_leg_joints(leg_id, angles)
"""

from stacc.gait.base import GaitGenerator
from stacc.gait.biped import BipedWalkGait
from stacc.gait.bound import BoundGait
from stacc.gait.cpg import CPGGenerator, CPGOscillator
from stacc.gait.crawl import CrawlGait
from stacc.gait.flight import FlightController
from stacc.gait.gallop import GallopGait
from stacc.gait.tripod import TripodGait
from stacc.gait.trot import TrotGait
from stacc.gait.walk import WalkGait

__all__ = [
    "GaitGenerator",
    "TrotGait",
    "WalkGait",
    "CrawlGait",
    "GallopGait",
    "BipedWalkGait",
    "BoundGait",
    "FlightController",
    "CPGGenerator",
    "CPGOscillator",
    "TripodGait",
]
