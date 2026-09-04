import os
from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="AMR_")

    HOST: str = "0.0.0.0"
    PORT: int = 8000
    CORS_ORIGINS: list[str] = [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:8000",
        "http://127.0.0.1:8000",
        "*",
    ]

    # Mode: "auto", "ros2", "ros1", "mock"
    BRIDGE_MODE: str = os.getenv("ROBOT_BRIDGE_MODE", "auto")

    # Static frontend directory
    STATIC_DIR: Path = Path(__file__).resolve().parent.parent / "static"

    # Streaming rates
    TELEMETRY_BROADCAST_HZ: float = 20.0
    STREAM_HEALTH_CHECK_HZ: float = 5.0


settings = Settings()

