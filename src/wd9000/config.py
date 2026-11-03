"""Load settings.yaml and secrets from .env."""

import os
from pathlib import Path

import yaml
from dotenv import load_dotenv

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SETTINGS = PROJECT_ROOT / "config" / "settings.yaml"


def load_settings(path: Path | None = None) -> dict:
    load_dotenv(PROJECT_ROOT / ".env")
    with open(path or DEFAULT_SETTINGS, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get_api_key() -> str | None:
    return os.environ.get("ROBOFLOW_API_KEY") or None
