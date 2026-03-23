import importlib
import math
import os
import xml.etree.ElementTree as ET

try:
    import yaml  # type: ignore
except ModuleNotFoundError:
    yaml = None

try:
    import pygame  # type: ignore
except ModuleNotFoundError:
    pygame = None


class SimpleVector2:
    def __init__(self, x=0.0, y=0.0):
        if hasattr(x, "x") and hasattr(x, "y") and y == 0.0:
            self.x = float(x.x)
            self.y = float(x.y)
            return
        if isinstance(x, (list, tuple)) and len(x) >= 2 and y == 0.0:
            self.x = float(x[0])
            self.y = float(x[1])
            return
        self.x = float(x)
        self.y = float(y)

    def __iter__(self):
        yield self.x
        yield self.y

    def __repr__(self):
        return f"SimpleVector2(x={self.x}, y={self.y})"


HAS_PYGAME = pygame is not None


def make_vector2(x=0.0, y=0.0):
    if HAS_PYGAME:
        return pygame.math.Vector2(x, y)
    return SimpleVector2(x, y)


def _strip_yaml_comment(line):
    in_single_quote = False
    in_double_quote = False

    for index, char in enumerate(line):
        if char == "'" and not in_double_quote:
            in_single_quote = not in_single_quote
        elif char == '"' and not in_single_quote:
            in_double_quote = not in_double_quote
        elif char == "#" and not in_single_quote and not in_double_quote:
            return line[:index]

    return line


def _parse_yaml_scalar(raw_value):
    value = raw_value.strip()
    if not value:
        return ""

    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]

    lowered = value.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if lowered in ("null", "none", "~"):
        return None

    if value.isdigit() or (value.startswith("-") and value[1:].isdigit()):
        return int(value)

    try:
        return float(value)
    except ValueError:
        return value


def _simple_yaml_safe_load(text):
    root = {}
    stack = [(-1, root)]

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        uncommented = _strip_yaml_comment(raw_line).rstrip()
        if not uncommented.strip():
            continue

        indent = len(uncommented) - len(uncommented.lstrip(" "))
        stripped = uncommented.strip()

        if stripped.startswith("- "):
            raise ValueError(
                f"Simple YAML fallback does not support list syntax (line {line_number})."
            )

        if ":" not in stripped:
            raise ValueError(f"Invalid YAML mapping on line {line_number}: {raw_line}")

        key, _, raw_value = stripped.partition(":")
        key = key.strip()
        raw_value = raw_value.strip()

        while stack and indent <= stack[-1][0]:
            stack.pop()

        if not stack:
            raise ValueError(f"Invalid indentation in YAML on line {line_number}.")

        parent = stack[-1][1]
        if raw_value == "":
            child = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _parse_yaml_scalar(raw_value)

    return root


def load_config(config_file):
    with open(config_file, "r", encoding="utf-8") as file_obj:
        raw_text = file_obj.read()

    if yaml is not None:
        return yaml.safe_load(raw_text)

    return _simple_yaml_safe_load(raw_text)


config = None


def set_config(config_file):
    global config
    config = load_config(config_file)
    config["config_file_path"] = config_file


def get_file_dirname(file_path):
    return os.path.dirname(os.path.abspath(file_path))


def parse_behavior_tree(xml_path):
    tree = ET.parse(xml_path)
    return tree.getroot()


def merge_dicts(dict1, dict2):
    merged_dict = dict1.copy()
    for key, value in dict2.items():
        if key in merged_dict:
            merged_dict[key] = max(merged_dict[key], value)
        else:
            merged_dict[key] = value
    return merged_dict


def convert_value(value):
    if value == "None":
        return None
    if isinstance(value, str):
        if value.isdigit() or (value.startswith("-") and value[1:].isdigit()):
            return int(value)
        try:
            return float(value)
        except ValueError:
            pass
    return value


class AttrDict(dict):
    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError as exc:
            raise AttributeError(key) from exc


def _is_vector2_instance(obj):
    if HAS_PYGAME and isinstance(obj, pygame.math.Vector2):
        return True
    return isinstance(obj, SimpleVector2)


def msg_serialize_default(obj):
    if isinstance(obj, set):
        return list(obj)
    if _is_vector2_instance(obj):
        return {"__v2__": True, "x": obj.x, "y": obj.y}
    if hasattr(obj, "__dict__"):
        return obj.__dict__
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


def msg_deserialize_hook(data):
    if "__v2__" in data:
        return make_vector2(data["x"], data["y"])
    return AttrDict(data)


def optional_import(name):
    if not name:
        return None
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as exc:
        if exc.name == name:
            return None
        raise
