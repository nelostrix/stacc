"""Universal AI modules for robotics.

Modules:
- vision: Computer vision — detection, features, tracking, depth
- slam: SLAM — localization and mapping
- nlp: Natural language — command parsing, speech interface
- path_planning: Path planning — A*, RRT*, potential fields
- behavior_tree: Behavior trees — task planning and decision making
"""

from stacc.ai.vision import (
    Detection,
    ObjectDetector,
    FeatureExtractor,
    OpticalFlow,
    ColorDetector,
    ArucoDetector,
    VisualOdometry,
    StereoDepthEstimator,
)

from stacc.ai.slam import (
    EKFSLAM,
    ParticleFilterSLAM,
    ICPMatcher,
    OccupancyGridMapper,
    SLAMNavigator,
)

from stacc.ai.nlp import (
    ParsedCommand,
    CommandParser,
    ActionMapper,
    SpeechInterface,
)

from stacc.ai.path_planning import (
    OccupancyGrid,
    AStarPlanner,
    DijkstraPlanner,
    RRTStarPlanner,
    RRTStarGridPlanner,
    PotentialFieldPlanner,
)

from stacc.ai.behavior_tree import (
    Status,
    Node,
    Action,
    Condition,
    Wait,
    Sequence,
    Selector,
    Parallel,
    Repeat,
    Invert,
    Timeout,
    Retry,
    AlwaysSucceed,
)

__all__ = [
    # vision
    "Detection",
    "ObjectDetector",
    "FeatureExtractor",
    "OpticalFlow",
    "ColorDetector",
    "ArucoDetector",
    "VisualOdometry",
    "StereoDepthEstimator",
    # slam
    "EKFSLAM",
    "ParticleFilterSLAM",
    "ICPMatcher",
    "OccupancyGridMapper",
    "SLAMNavigator",
    # nlp
    "ParsedCommand",
    "CommandParser",
    "ActionMapper",
    "SpeechInterface",
    # path_planning
    "OccupancyGrid",
    "AStarPlanner",
    "DijkstraPlanner",
    "RRTStarPlanner",
    "RRTStarGridPlanner",
    "PotentialFieldPlanner",
    # behavior_tree
    "Status",
    "Node",
    "Action",
    "Condition",
    "Wait",
    "Sequence",
    "Selector",
    "Parallel",
    "Repeat",
    "Invert",
    "Timeout",
    "Retry",
    "AlwaysSucceed",
]
