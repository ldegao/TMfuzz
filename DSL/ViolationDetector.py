from DSL.RuleChecker import AgainstRules
import DSL as sd
import config
from collections import defaultdict

class Violation(object):
    def __init__(self) -> None:
        self.violation_dict = defaultdict(lambda: dict()) 

    def check_violation(self):
        if_violate = False
        id_list = sd._state.npc_id
        id_list.append(sd._state.ego_id)
        for id in id_list:
            state = sd._state.get_state_by_id(id)
            # 确保有足够的数据进行检查
            if len(state.transforms) < 2:
                continue
            # 调整采样间隔，确保不会超出范围
            max_index = len(state.transforms) - 1
            for i in range(0, max_index + 1, 20):
                # 确保索引在有效范围内
                if i > max_index:
                    break
                againstRules = AgainstRules(state)
                rules = againstRules.check(index= i)
                if len(rules) > 0:
                    self.violation_dict[sd._vehicleDict.name(id)][i] = ';'.join(rules)
                    if id == sd._state.ego_id:
                        if_violate = True
        return if_violate