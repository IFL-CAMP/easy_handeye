"""Shared pytest fixtures and configuration for all tests."""

import os
import tempfile
from pathlib import Path
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def temp_dir():
    """Create a temporary directory for test files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


@pytest.fixture
def mock_config():
    """Provide a mock configuration object."""
    config = MagicMock()
    config.debug = False
    config.verbose = False
    config.timeout = 30
    return config


@pytest.fixture
def sample_calibration_data():
    """Provide sample calibration data for testing."""
    return {
        "camera_matrix": [[1000, 0, 320], [0, 1000, 240], [0, 0, 1]],
        "distortion_coeffs": [0.1, -0.2, 0, 0, 0],
        "rotation_vector": [0.1, 0.2, 0.3],
        "translation_vector": [0.1, 0.2, 0.3]
    }


@pytest.fixture
def mock_ros_node():
    """Mock a ROS node for testing without ROS dependencies."""
    node = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    node.create_publisher = MagicMock()
    node.create_subscription = MagicMock()
    node.create_service = MagicMock()
    node.create_client = MagicMock()
    return node


@pytest.fixture
def sample_transformation_matrix():
    """Provide a sample 4x4 transformation matrix."""
    import numpy as np
    return np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.2],
        [0, 0, 1, 0.3],
        [0, 0, 0, 1]
    ])


@pytest.fixture(autouse=True)
def reset_environment():
    """Reset environment variables before each test."""
    original_env = os.environ.copy()
    yield
    os.environ.clear()
    os.environ.update(original_env)


@pytest.fixture
def mock_file_system(tmp_path):
    """Create a mock file system structure for testing."""
    structure = {
        "config": tmp_path / "config",
        "data": tmp_path / "data",
        "logs": tmp_path / "logs",
        "cache": tmp_path / "cache"
    }
    
    for directory in structure.values():
        directory.mkdir(parents=True, exist_ok=True)
    
    return structure


@pytest.fixture
def capture_logs():
    """Capture log messages for assertion in tests."""
    logs = []
    
    class LogCapture:
        def __init__(self):
            self.logs = logs
        
        def debug(self, msg):
            self.logs.append(("DEBUG", msg))
        
        def info(self, msg):
            self.logs.append(("INFO", msg))
        
        def warning(self, msg):
            self.logs.append(("WARNING", msg))
        
        def error(self, msg):
            self.logs.append(("ERROR", msg))
        
        def clear(self):
            self.logs.clear()
    
    return LogCapture()


@pytest.fixture
def sample_robot_pose():
    """Provide sample robot pose data."""
    return {
        "position": {"x": 0.5, "y": 0.3, "z": 0.7},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }


@pytest.fixture
def sample_camera_info():
    """Provide sample camera info data."""
    return {
        "width": 640,
        "height": 480,
        "K": [1000, 0, 320, 0, 1000, 240, 0, 0, 1],
        "D": [0.1, -0.2, 0, 0, 0],
        "R": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "P": [1000, 0, 320, 0, 0, 1000, 240, 0, 0, 0, 1, 0]
    }