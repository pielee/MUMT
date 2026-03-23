import os

from modules.utils import (
    convert_value,
    get_file_dirname,
    optional_import,
    parse_behavior_tree,
)


def build_behavior_tree(agent, behavior_tree_xml: str, env_pkg: str):
    bt_module = optional_import(f"{env_pkg}.bt_nodes")
    mission_bt_module = optional_import(f"{env_pkg}.mission_bt_nodes")

    if bt_module is None:
        raise ModuleNotFoundError(
            f"[ERROR] Could not import '{env_pkg}.bt_nodes'. "
            "Make sure your environment package exposes bt_nodes."
        )

    xml_root = parse_behavior_tree(behavior_tree_xml)
    behavior_tree_root = xml_root.find("BehaviorTree")
    if behavior_tree_root is None:
        raise ValueError(
            f"[ERROR] XML file '{behavior_tree_xml}' does not contain a <BehaviorTree> root."
        )

    _validate_xml_tree(
        behavior_tree_root,
        bt_module=bt_module,
        top_xml_path=behavior_tree_xml,
    )

    return _parse_xml_to_bt(
        behavior_tree_root,
        bt_module=bt_module,
        mission_bt_module=mission_bt_module,
        agent=agent,
        top_xml_path=behavior_tree_xml,
    )


def _require_bt_class(bt_module, node_type, category):
    node_class = getattr(bt_module, node_type, None)
    if node_class is None:
        raise AttributeError(
            f"[ERROR] '{node_type}' is referenced as a {category} node, "
            f"but '{bt_module.__name__}' does not export that class."
        )
    return node_class


def _validate_xml_tree(xml_node, *, bt_module, top_xml_path):
    node_type = xml_node.tag
    bt_node_list = getattr(bt_module, "BTNodeList")

    if node_type == "BehaviorTree":
        for child in xml_node:
            _validate_xml_tree(child, bt_module=bt_module, top_xml_path=top_xml_path)
        return

    if node_type == "SubTree":
        subtree_id = xml_node.attrib.get("ID")
        if not subtree_id:
            raise ValueError("[ERROR] SubTree node must have an 'ID' attribute")

        base_dir = get_file_dirname(top_xml_path)
        sub_behavior_tree_xml = os.path.join(base_dir, f"{subtree_id}.xml")
        subtree_root = parse_behavior_tree(sub_behavior_tree_xml)
        subtree_behavior_root = subtree_root.find("BehaviorTree")
        if subtree_behavior_root is None:
            raise ValueError(
                f"[ERROR] SubTree XML '{sub_behavior_tree_xml}' does not contain a <BehaviorTree> root."
            )
        _validate_xml_tree(
            subtree_behavior_root,
            bt_module=bt_module,
            top_xml_path=sub_behavior_tree_xml,
        )
        return

    if node_type in bt_node_list.CONTROL_NODES:
        _require_bt_class(bt_module, node_type, "control")
        for child in xml_node:
            _validate_xml_tree(child, bt_module=bt_module, top_xml_path=top_xml_path)
        return

    if node_type in bt_node_list.DECORATOR_NODES:
        _require_bt_class(bt_module, node_type, "decorator")
        if len(list(xml_node)) != 1:
            raise ValueError(f"[ERROR] Decorator '{node_type}' must have exactly 1 child.")
        for child in xml_node:
            _validate_xml_tree(child, bt_module=bt_module, top_xml_path=top_xml_path)
        return

    if node_type in (bt_node_list.ACTION_NODES + bt_node_list.CONDITION_NODES):
        _require_bt_class(bt_module, node_type, "leaf")
        return

    raise ValueError(f"[ERROR] Unknown behavior node type in XML: {node_type}")


def _parse_xml_to_bt(xml_node, *, bt_module, mission_bt_module, agent, top_xml_path):
    node_type = xml_node.tag

    if node_type == "SubTree":
        subtree_id = xml_node.attrib.get("ID")
        if not subtree_id:
            raise ValueError("[ERROR] SubTree node must have an 'ID' attribute")

        base_dir = get_file_dirname(top_xml_path)
        sub_behavior_tree_xml = os.path.join(base_dir, f"{subtree_id}.xml")
        subtree_root = parse_behavior_tree(sub_behavior_tree_xml)

        return _parse_xml_to_bt(
            subtree_root.find("BehaviorTree"),
            bt_module=bt_module,
            mission_bt_module=mission_bt_module,
            agent=agent,
            top_xml_path=sub_behavior_tree_xml,
        )

    children = [
        _parse_xml_to_bt(
            child,
            bt_module=bt_module,
            mission_bt_module=mission_bt_module,
            agent=agent,
            top_xml_path=top_xml_path,
        )
        for child in xml_node
    ]

    bt_node_list = getattr(bt_module, "BTNodeList")
    attrib = {key: convert_value(value) for key, value in xml_node.attrib.items()}

    if node_type in bt_node_list.CONTROL_NODES:
        control_class = _require_bt_class(bt_module, node_type, "control")
        return control_class(node_type, children=children, **attrib)

    if node_type in bt_node_list.DECORATOR_NODES:
        decorator_class = _require_bt_class(bt_module, node_type, "decorator")
        if len(children) != 1:
            raise ValueError(f"[ERROR] Decorator '{node_type}' must have exactly 1 child.")
        return decorator_class(node_type, child=children[0], **attrib)

    if node_type in (bt_node_list.ACTION_NODES + bt_node_list.CONDITION_NODES):
        action_class = _require_bt_class(bt_module, node_type, "leaf")
        return action_class(node_type, agent, **attrib)

    if node_type == "BehaviorTree":
        if not children:
            raise ValueError("[ERROR] <BehaviorTree> has no child node.")
        return children[0]

    raise ValueError(f"[ERROR] Unknown behavior node type: {node_type}")
