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

        tick_index = int(self.blackboard.get('bt_tick_index', 0)) + 1
        self.blackboard['bt_tick_index'] = tick_index
        self.blackboard['tick_dt'] = dt
        self.blackboard['runtime_interface'] = self.interface
        self.blackboard['bt_tick_condition_trace'] = []
        self.blackboard['bt_tick_branch_attempts'] = []
        self.blackboard['bt_tick_overwrites'] = []
        self.blackboard['bt_tick_final_command'] = None
        self.blackboard['bt_tick_interface_final_command'] = None
        self.blackboard['bt_tick_current_branch'] = ''
        self.blackboard['bt_tick_tree_status'] = ''

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
        result = await self.tree.run(self, self.blackboard)
        self.blackboard['bt_tick_tree_status'] = getattr(result, 'name', str(result))
        self._maybe_print_bt_debug_summary()
        return result

    def _maybe_print_bt_debug_summary(self):
        debug_cfg = config.get('f16_ai', {}).get('debug', {})
        if not debug_cfg.get('enabled', False):
            return

        tick_index = int(self.blackboard.get('bt_tick_index', 0))
        log_every_n_ticks = max(1, int(debug_cfg.get('log_every_n_ticks', 1)))
        overwrites = self.blackboard.get('bt_tick_overwrites', [])
        final_command = self.blackboard.get('bt_tick_final_command') or {}
        interface_final_command = self.blackboard.get('bt_tick_interface_final_command') or {}
        if (tick_index % log_every_n_ticks) != 0 and not overwrites:
            return

        ego = self.blackboard.get('ego_state')
        if ego is None:
            ego_text = "ego=missing"
        else:
            ego_text = (
                f"ego(on_ground={getattr(ego, 'on_ground', 'NA')} "
                f"airspeed={getattr(ego, 'airspeed_mps', 0.0):.1f} "
                f"gs={getattr(ego, 'ground_speed_mps', 0.0):.1f} "
                f"vel=({getattr(getattr(ego, 'velocity_mps', None), 'x', 0.0):.1f},"
                f"{getattr(getattr(ego, 'velocity_mps', None), 'y', 0.0):.1f},"
                f"{getattr(getattr(ego, 'velocity_mps', None), 'z', 0.0):.1f}) "
                f"alt={getattr(ego, 'altitude_world_m', 0.0):.1f} "
                f"att=({getattr(ego, 'yaw_deg', 0.0):.1f},"
                f"{getattr(ego, 'pitch_deg', 0.0):.1f},"
                f"{getattr(ego, 'roll_deg', 0.0):.1f}))"
            )

        condition_trace = self.blackboard.get('bt_tick_condition_trace', [])
        branch_attempts = self.blackboard.get('bt_tick_branch_attempts', [])
        print(
            "[F16BT:TICK] "
            f"tick={tick_index} status={self.blackboard.get('bt_tick_tree_status', 'UNKNOWN')} "
            f"current_branch={self.blackboard.get('bt_tick_current_branch', 'NONE') or 'NONE'} "
            f"takeoff_ticks={self.blackboard.get('f16_takeoff_active_ticks', 0)} "
            f"force_takeoff={self.blackboard.get('f16_force_takeoff_active', False)} "
            f"final_command={final_command} "
            f"interface_final_command={interface_final_command} "
            f"conditions={condition_trace} "
            f"branch_attempts={branch_attempts} "
            f"overwrites={overwrites} "
            f"{ego_text}"
        )

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
