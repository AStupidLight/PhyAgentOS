"""Placeholder SAM worker for the simplified in-repo navigation SDK."""

from __future__ import annotations

import json
import sys


def main() -> int:
    payload = {"status": "not_implemented", "message": "sam3 worker is not bundled in the simplified navigation SDK"}
    sys.stdout.write(json.dumps(payload) + "\n")
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
