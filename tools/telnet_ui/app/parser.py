import re
from dataclasses import dataclass
from typing import Dict, List, Tuple


_TAG_RE = re.compile(r"^\[([A-Z]+)\]")
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
        body_without_groups = self._extract_group_fields(body, fields)
        self._extract_key_values(body_without_groups, fields)

        state_match = re.match(r"^(ON|OFF)\b", body.strip(), flags=re.IGNORECASE)
        if state_match:
            self._store_field(fields, "state", state_match.group(1).upper())

        return fields

    def _extract_message(self, body: str) -> str:
        clean = self._strip_group_blocks(body)
        clean = _KEY_VALUE_RE.sub("", clean)
        clean = re.sub(r"\s+", " ", clean).strip()
        return clean

    def _store_field(self, fields: Dict[str, str], key: str, value: str) -> None:
        if key not in fields:
            fields[key] = value
            return

        dup_idx = 2
        while True:
            dup_key = f"{key}__dup{dup_idx}"
            if dup_key not in fields:
                fields[dup_key] = value
                return
            dup_idx += 1

    def _extract_key_values(self, text: str, fields: Dict[str, str], prefix: str = "") -> None:
        for key, value in _KEY_VALUE_RE.findall(text):
            full_key = f"{prefix}.{key}" if prefix else key
            self._store_field(fields, full_key, value)

    def _extract_group_fields(self, text: str, fields: Dict[str, str], prefix: str = "") -> str:
        result_chars: List[str] = []
        i = 0
        length = len(text)
        while i < length:
            if text[i].isalnum() or text[i] == "_":
                token_start = i
                while i < length and (text[i].isalnum() or text[i] == "_"):
                    i += 1
                token = text[token_start:i]
                if i < length and text[i] == "{":
                    brace_end = self._find_matching_brace(text, i)
                    if brace_end != -1:
                        group_prefix = f"{prefix}.{token}" if prefix else token
                        inner = text[i + 1:brace_end]
                        inner_without_groups = self._extract_group_fields(inner, fields, group_prefix)
                        self._extract_key_values(inner_without_groups, fields, group_prefix)
                        i = brace_end + 1
                        continue
                result_chars.append(text[token_start:i])
                continue

            result_chars.append(text[i])
            i += 1

        return "".join(result_chars)

    def _strip_group_blocks(self, text: str) -> str:
        result_chars: List[str] = []
        i = 0
        length = len(text)
        while i < length:
            if text[i].isalnum() or text[i] == "_":
                token_start = i
                while i < length and (text[i].isalnum() or text[i] == "_"):
                    i += 1
                if i < length and text[i] == "{":
                    brace_end = self._find_matching_brace(text, i)
                    if brace_end != -1:
                        i = brace_end + 1
                        continue
                result_chars.append(text[token_start:i])
                continue

            result_chars.append(text[i])
            i += 1

        return "".join(result_chars)

    def _find_matching_brace(self, text: str, open_idx: int) -> int:
        if open_idx >= len(text) or text[open_idx] != "{":
            return -1
        depth = 0
        for idx in range(open_idx, len(text)):
            ch = text[idx]
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return idx
        return -1

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
        if head == "SYS":
            return "system"
        if head in ("PI", "DRIVE", "RC"):
            return "logs"

        return "unknown"
