"""Prepare a temporary, prop-free world for mapping the static environment."""
from pathlib import Path
import re
import tempfile


def without_scenario_props(world_path):
    """Return a temporary world path, or None if this world has no prop section.

    The source stays unchanged. WebotsLauncher copies this again for its own
    supervisor, so local EXTERNPROTO paths must be absolute in this intermediate
    file. Factory textures and other upstream assets already use absolute URLs.
    """
    source = Path(world_path)
    text = source.read_text()
    begin = '# BEGIN SCENARIO_PROPS'
    end = '# END SCENARIO_PROPS'
    if begin not in text and end not in text:
        return None
    if text.count(begin) != 1 or text.count(end) != 1:
        raise ValueError('Expected exactly one complete SCENARIO_PROPS section')
    prefix, remainder = text.split(begin)
    _, suffix = remainder.split(end)
    text = prefix + '# Scenario props omitted for static mapping.\n' + suffix

    def absolute_proto(match):
        path = match.group(1)
        if '://' in path or Path(path).is_absolute():
            return match.group(0)
        return f'EXTERNPROTO "{(source.parent / path).resolve()}"'

    text = re.sub(r'EXTERNPROTO\s+"([^"]+)"', absolute_proto, text)
    with tempfile.NamedTemporaryFile(mode='w', suffix='_limo_mapping.wbt', delete=False) as output:
        output.write(text)
        return output.name
