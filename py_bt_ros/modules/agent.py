import time

from modules.bt_constructor import build_behavior_tree
from modules.utils import config, make_vector2, optional_import


class Agent:
    def __init__(self, ros_namespace=None):
        self.blackboard = {}
        self.ros_namespace = ros_namespace
        self.type = config['agent'].get('type', None)

        self.messages_received = []
        self.ros_bridge = None

        # Preserved from the original project so scenario code can still refer
        # to a stable agent id even though the runtime is no longer ROS-based.
        self.agent_id = ros_namespace.strip('/') if ros_namespace else "no_id_agent"
        self.message_to_share = {}
        self.position = make_vector2(0, 0)

        self.interface = self._create_runtime_interface()
        self._last_tick_timestamp = None

    def _create_runtime_interface(self):
        runtime_cfg = config.get('runtime', {})
        interface_path = runtime_cfg.get('interface_class')
        if not interface_path:
            return None

        module_name, class_name = interface_path.rsplit('.', 1)
        interface_module = optional_import(module_name)
        if interface_module is None:
            raise ModuleNotFoundError(
                f"[ERROR] Could not import runtime interface module '{module_name}'."
            )

        interface_class = getattr(interface_module, class_name, None)
        if interface_class is None:
            raise AttributeError(
                f"[ERROR] Runtime interface class '{class_name}' was not found in '{module_name}'."
            )

        return interface_class(config=config, agent=self)

    def create_behavior_tree(self, behavior_tree_xml):
        self.behavior_tree_xml = behavior_tree_xml
        scenario_package = config.get('scenario', {}).get('environment')
        self.tree = build_behavior_tree(self, behavior_tree_xml, scenario_package)

    def _reset_bt_action_node_status(self):
        self.tree.reset()

    def _update_runtime_blackboard(self):
        now = time.monotonic()
        dt = 0.0 if self._last_tick_timestamp is None else max(0.0, now - self._last_tick_timestamp)
        self._last_tick_timestamp = now

        self.blackboard['tick_dt'] = dt
        self.blackboard['runtime_interface'] = self.interface

        if self.interface is None:
            return

        if hasattr(self.interface, 'tick'):
            self.interface.tick(dt)

        if hasattr(self.interface, 'get_ego_state'):
            ego_state = self.interface.get_ego_state()
            self.blackboard['ego_state'] = ego_state
            if hasattr(ego_state, 'position_m'):
                self.position = make_vector2(ego_state.position_m.x, ego_state.position_m.y)

        if hasattr(self.interface, 'get_enemy_state'):
            self.blackboard['enemy_state'] = self.interface.get_enemy_state()

    async def run_tree(self):
        self._reset_bt_action_node_status()
        self._update_runtime_blackboard()
        return await self.tree.run(self, self.blackboard)

    def reset_messages_received(self):
        self.messages_received = []

    def set_planned_tasks(self, *_):
        pass

    def halt_tree(self):
        if not hasattr(self, 'tree'):
            return

        def _halt(node):
            if hasattr(node, 'children'):
                for child in node.children:
                    _halt(child)
            node.halt()

        _halt(self.tree)

    def close(self):
        self.halt_tree()
        if self.interface and hasattr(self.interface, 'close'):
            self.interface.close()
