"""Minimal YAML parser for 3Dblox format (no external dependencies)."""
import re
from typing import Any, Dict, List, Union


def safe_load(text: str) -> Union[Dict, List, None]:
    """Parse a simple YAML subset used by 3Dblox files."""
    lines = text.splitlines()
    _, result = _parse_block(lines, 0, 0)
    return result


def _parse_block(lines, start_idx, base_indent):
    """Parse a dictionary block at a given indentation level."""
    result: Dict[str, Any] = {}
    i = start_idx

    while i < len(lines):
        line = lines[i]
        stripped = line.lstrip()

        if not stripped or stripped.startswith('#'):
            i += 1
            continue

        indent = len(line) - len(stripped)

        if indent < base_indent:
            break

        if indent > base_indent:
            # Skip deeper lines - they belong to a nested block already consumed
            i += 1
            continue

        # List item at this level - stop dict parsing, let parent handle it
        if stripped.startswith('- '):
            break

        # Key-value pair
        if ':' in stripped:
            key, value = _split_first(stripped, ':')
            key = key.strip()
            value = value.strip() if value else None

            if value:
                result[key] = _convert_value(value)
                i += 1
            else:
                # Look ahead to determine if this is a list, nested dict, or None
                i, child = _parse_child(lines, i + 1, indent)
                result[key] = child
            continue

        # Implicit key (bare string without ':', followed by nested content)
        if stripped and not stripped.startswith('- ') and ':' not in stripped:
            look = i + 1
            while look < len(lines):
                look_stripped = lines[look].lstrip()
                if look_stripped and not look_stripped.startswith('#'):
                    break
                look += 1
            if look < len(lines):
                look_indent = len(lines[look]) - len(lines[look].lstrip())
                if look_indent > indent:
                    key = stripped
                    next_idx, nested = _parse_block(lines, i + 1, look_indent)
                    if nested:
                        result[key] = nested
                    else:
                        result[key] = None
                    i = next_idx
                    continue

        i += 1

    return i, result


def _parse_child(lines, start_idx, parent_indent):
    """Parse the child of a key (could be a list, a dict, or None)."""
    i = start_idx

    # Skip empty lines and comments
    while i < len(lines):
        stripped = lines[i].lstrip()
        if stripped and not stripped.startswith('#'):
            break
        i += 1

    if i >= len(lines):
        return i, None

    first_stripped = lines[i].lstrip()
    first_indent = len(lines[i]) - len(first_stripped)

    if first_indent < parent_indent:
        # Not a child, just a sibling or parent
        return i, None

    # If it's a list item, parse as list
    if first_stripped.startswith('- '):
        return _parse_list(lines, i, first_indent)

    # Otherwise, parse as nested dict
    return _parse_block(lines, i, first_indent)


def _parse_list(lines, start_idx, list_indent):
    """Parse a list of items at a given indentation level."""
    items = []
    i = start_idx

    while i < len(lines):
        line = lines[i]
        stripped = line.lstrip()

        if not stripped or stripped.startswith('#'):
            i += 1
            continue

        indent = len(line) - len(stripped)

        if indent < list_indent:
            break

        if indent > list_indent:
            # Skip deeper lines - they belong to nested content already consumed
            i += 1
            continue

        if not stripped.startswith('- '):
            break

        item_text = stripped[2:].strip()

        if ':' in item_text and not item_text.startswith('['):
            # Nested dict in list item
            key, value = _split_first(item_text, ':')
            key = key.strip()
            value = value.strip() if value else None

            if value:
                # Simple inline value: `- key: value`
                items.append({key: _convert_value(value)})
                i += 1
            else:
                # Complex item with nested content
                next_idx, nested = _parse_child(lines, i + 1, indent)
                items.append({key: nested})
                i = next_idx
        else:
            # Simple value: `- value`
            items.append(_convert_value(item_text))
            i += 1

    return i, items


def _split_first(s: str, sep: str) -> tuple:
    idx = s.find(sep)
    if idx == -1:
        return s, None
    return s[:idx], s[idx + len(sep):]


def _convert_value(v: str) -> Any:
    v = v.strip()
    if not v:
        return None

    # Boolean
    if v.lower() == 'true':
        return True
    if v.lower() == 'false':
        return False

    # Null
    if v.lower() in ('null', '~', ''):
        return None

    # List literal like [1, 2, 3]
    if v.startswith('[') and v.endswith(']'):
        inner = v[1:-1]
        if not inner.strip():
            return []
        return [_convert_value(x.strip()) for x in inner.split(',')]

    # String with quotes
    if (v.startswith('"') and v.endswith('"')) or (v.startswith("'") and v.endswith("'")):
        return v[1:-1]

    # Number
    try:
        if '.' in v or 'e' in v.lower():
            return float(v)
        return int(v)
    except ValueError:
        pass

    # String
    return v
