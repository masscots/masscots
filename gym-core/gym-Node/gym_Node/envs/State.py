import enum

class NodeState(enum.Enum):
    Loading = 0
    Free = 1
    Tip = 2
    Extra = 3
    Orphan = 4
    Backbone = 5
    Origin = 6
    Destination = 7
    Tentacle = 8
    Reinforce = 9

class TentacleState(enum.Enum):
    Forming = 1
    Complete = 2
    Next_Destination = 3
    Reinforcing = 4
    Damaged = 5

class EtcState(enum.Enum):
    Unknown = -9999