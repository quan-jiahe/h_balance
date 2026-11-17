"""Download and verify the selected upstream Model2; run on PC, not on K230."""

import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    project = Path(__file__).resolve().parents[1]
    model = json.loads((project / "k230/model_manifest.json").read_text(encoding="utf-8"))
    target = project / "k230/kmodel" / model["filename"]
    if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == model["sha256"]:
        print("Model already verified:", target)
        return
    with urllib.request.urlopen(model["url"], timeout=60) as response:
        data = response.read()
    if len(data) != model["size"] or hashlib.sha256(data).hexdigest() != model["sha256"]:
        raise ValueError("Model size/hash mismatch; existing file was not changed")
    target.parent.mkdir(parents=True, exist_ok=True)
    staged = target.with_suffix(".download")
    staged.write_bytes(data)
    staged.replace(target)
    print("Model downloaded and verified:", target)


if __name__ == "__main__":
    main()
