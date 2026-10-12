# Locally modified for AUBO, 2026-10-12. See LOCAL_CHANGES.md; Apache-2.0.
"""Optional LeRobot facade; the original loop lives in core.quest_teleop."""
from dataclasses import dataclass
from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from ..core.quest_teleop import QuestTeleoperator, QuestTeleoperatorConfig

@TeleoperatorConfig.register_subclass("bi_quest_teleop")
@dataclass
class BiQuestTeleoperatorConfig(QuestTeleoperatorConfig, TeleoperatorConfig):
    pass

class BiQuestTeleoperator(QuestTeleoperator, Teleoperator):
    config_class = BiQuestTeleoperatorConfig
    name = "bi_quest_teleop"

    def __init__(self, config):
        Teleoperator.__init__(self, config)
        QuestTeleoperator.__init__(self, config)
