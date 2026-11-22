def make_backend(kind: str):
    if kind == "mock":
        from .mock import MockBackend
        return MockBackend()
    if kind == "gazebo":
        from .gz import GazeboBackend
        return GazeboBackend()
    raise ValueError(f"unknown SIM_BACKEND {kind!r} (use gazebo|mock)")
