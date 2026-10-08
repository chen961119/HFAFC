"""Generate local VS Code debug entries after any PlatformIO build.

The templates are tracked; .vscode is ignored. Existing unrelated entries stay intact.
This file can also be run directly with PlatformIO's Python interpreter.
"""

import json
from pathlib import Path


if "Import" in globals():
    Import("env")
    ROOT = Path(env.subst("$PROJECT_DIR")).resolve()
else:
    ROOT = Path(__file__).resolve().parents[2]
TEMPLATES = ROOT / "tools" / "teensy_debug"


def parse_jsonc(source):
    """Accept VS Code comments and trailing commas without extra packages."""
    stripped = []
    index = 0
    quoted = False
    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""
        if quoted:
            stripped.append(char)
            if char == "\\" and next_char:
                stripped.append(next_char)
                index += 1
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
            stripped.append(char)
        elif char == "/" and next_char == "/":
            index += 2
            while index < len(source) and source[index] not in "\r\n":
                index += 1
            continue
        elif char == "/" and next_char == "*":
            index += 2
            while index + 1 < len(source) and source[index:index + 2] != "*/":
                index += 1
            index += 2
            continue
        else:
            stripped.append(char)
        index += 1
    without_comments = "".join(stripped)
    cleaned = []
    quoted = False
    index = 0
    while index < len(without_comments):
        char = without_comments[index]
        if char == '"':
            quoted = not quoted
        if char == "\\" and quoted and index + 1 < len(without_comments):
            cleaned.extend((char, without_comments[index + 1]))
            index += 2
            continue
        if char == "," and not quoted:
            next_index = index + 1
            while next_index < len(without_comments) and without_comments[next_index].isspace():
                next_index += 1
            if next_index < len(without_comments) and without_comments[next_index] in "}]":
                index += 1
                continue
        cleaned.append(char)
        index += 1
    return json.loads("".join(cleaned))


def read_local(path, collection):
    if not path.exists():
        return {}
    source = path.read_text(encoding="utf-8-sig")
    try:
        value = parse_jsonc(source)
    except json.JSONDecodeError as error:
        raise ValueError(f"Cannot update {path}: invalid JSON at line {error.lineno}") from error
    if not isinstance(value, dict) or not isinstance(value.get(collection, []), list):
        raise ValueError(f"Cannot update {path}: expected a {collection} list")
    return value


def local_or_backup(path, collection):
    try:
        return read_local(path, collection)
    except ValueError as error:
        backup = path.with_name(path.name + ".invalid-backup")
        suffix = 1
        while backup.exists():
            backup = path.with_name(path.name + f".invalid-backup-{suffix}")
            suffix += 1
        backup.write_bytes(path.read_bytes())
        print(f"{error}; previous contents saved to {backup}")
        return {}


def merge_file(filename, collection, identity):
    local = ROOT / ".vscode" / filename
    template = json.loads((TEMPLATES / f"vscode_{filename}").read_text(encoding="utf-8"))
    existing = local_or_backup(local, collection)
    incoming = template[collection]
    names = {item.get(identity) for item in incoming}
    retained = [item for item in existing.get(collection, [])
                if item.get(identity) not in names]
    existing.update({key: value for key, value in template.items() if key != collection})
    existing[collection] = incoming + retained
    rendered = json.dumps(existing, ensure_ascii=False, indent=2) + "\n"
    local.parent.mkdir(parents=True, exist_ok=True)
    if not local.exists() or local.read_text(encoding="utf-8-sig") != rendered:
        local.write_text(rendered, encoding="utf-8")


def configure():
    merge_file("launch.json", "configurations", "name")
    merge_file("tasks.json", "tasks", "label")
    local = ROOT / ".vscode" / "extensions.json"
    existing = local_or_backup(local, "recommendations")
    template = json.loads((TEMPLATES / "vscode_extensions.json").read_text(encoding="utf-8"))
    recommendations = existing.get("recommendations", [])
    for extension in template["recommendations"]:
        if extension not in recommendations:
            recommendations.append(extension)
    existing["recommendations"] = recommendations
    rendered = json.dumps(existing, ensure_ascii=False, indent=2) + "\n"
    if not local.exists() or local.read_text(encoding="utf-8-sig") != rendered:
        local.write_text(rendered, encoding="utf-8")


if "Import" in globals():
    configure()
elif __name__ == "__main__":
    configure()
