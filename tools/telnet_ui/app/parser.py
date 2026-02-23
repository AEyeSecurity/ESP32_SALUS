import re
from dataclasses import dataclass
from typing import Dict, List, Tuple


_TAG_RE = re.compile(r"^\[([A-Z]+)\]")
_BRACE_GROUP_RE = re.compile(r"([A-Za-z0-9_]+)\{([^}]*)\}")
_KEY_VALUE_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


@dataclass
class ParsedEvent:
    line: str
    tags: List[str]
    section: str
    fields: Dict[str, str]
    message: str


class TelnetLineParser:
    def parse_line(self, line: str) -> ParsedEvent:
        text = line.strip()
        tags, body = self._extract_tags(text)
        fields = self._extract_fields(body)
        message = self._extract_message(body)
        section = self._resolve_section(tags)

        if section == "streams":
            if tags and tags[0] == "SPD":
                fields.setdefault("stream", "speed")
            elif tags and tags[0] == "SPID":
                fields.setdefault("stream", "spid")

        return ParsedEvent(line=text, tags=tags, section=section, fields=fields, message=message)

    def _extract_tags(self, line: str) -> Tuple[List[str], str]:
        tags: List[str] = []
        remainder = line
        while remainder.startswith("["):
            match = _TAG_RE.match(remainder)
            if not match:
                break
            tags.append(match.group(1))
            remainder = remainder[match.end():]
        return tags, remainder.strip()

    def _extract_fields(self, body: str) -> Dict[str, str]:
        fields: Dict[str, str] = {}

        for match in _BRACE_GROUP_RE.finditer(body):
            group_name = match.group(1)
            group_content = match.group(2)
            for key, value in _KEY_VALUE_RE.findall(group_content):
                fields["%s.%s" % (group_name, key)] = value

        body_without_groups = _BRACE_GROUP_RE.sub("", body)
        for key, value in _KEY_VALUE_RE.findall(body_without_groups):
            fields[key] = value

        state_match = re.match(r"^(ON|OFF)\b", body.strip(), flags=re.IGNORECASE)
        if state_match:
            fields.setdefault("state", state_match.group(1).upper())

        return fields

    def _extract_message(self, body: str) -> str:
        clean = _BRACE_GROUP_RE.sub("", body)
        clean = _KEY_VALUE_RE.sub("", clean)
        clean = re.sub(r"\s+", " ", clean).strip()
        return clean

    def _resolve_section(self, tags: List[str]) -> str:
        if not tags:
            return "unknown"

        head = tags[0]
        tail = tags[1] if len(tags) > 1 else ""

        if head == "NET":
            return "net"
        if head == "STEER":
            return "steer"
        if head == "PID":
            return "pid"
        if head == "SPID" and tail == "STREAM":
            return "streams"
        if head == "SPID":
            return "spid"
        if head == "SPD" and tail == "STREAM":
            return "streams"
        if head == "SPD" and tail == "STATUS":
            return "speed"
        if head == "SPD":
            return "speed"
        if head == "PI" and tail == "STATUS":
            return "comms"
        if head == "PI" and tail in ("RX", "TX"):
            return "comms"
        if head == "PI" and tail == "RX" and len(tags) > 2 and tags[2] == "WARN":
            return "comms"
        if head == "DRIVE" and tail == "LOG":
            return "drive_log"
        if head in ("PI", "DRIVE", "RC"):
            return "logs"

        return "unknown"
