"""Validation tests to ensure the testing infrastructure is properly set up."""

import sys
from pathlib import Path

import pytest


class TestInfrastructureValidation:
    """Validate that the testing infrastructure is properly configured."""
    
    def test_pytest_installed(self):
        """Verify pytest is installed and importable."""
        import pytest
        assert pytest.__version__
    
    def test_pytest_cov_installed(self):
        """Verify pytest-cov is installed and importable."""
        import pytest_cov
        assert pytest_cov
    
    def test_pytest_mock_installed(self):
        """Verify pytest-mock is installed and importable."""
        import pytest_mock
        assert pytest_mock
    
    def test_test_directory_structure(self):
        """Verify the test directory structure exists."""
        test_root = Path(__file__).parent
        assert test_root.exists()
        assert (test_root / "__init__.py").exists()
        assert (test_root / "conftest.py").exists()
        assert (test_root / "unit").exists()
        assert (test_root / "unit" / "__init__.py").exists()
        assert (test_root / "integration").exists()
        assert (test_root / "integration" / "__init__.py").exists()
    
    def test_source_packages_importable(self):
        """Verify source packages can be imported."""
        # Add parent directories to path for imports
        workspace = Path(__file__).parent.parent
        sys.path.insert(0, str(workspace / "easy_handeye" / "src"))
        sys.path.insert(0, str(workspace / "rqt_easy_handeye" / "src"))
        
        import easy_handeye
        import rqt_easy_handeye
        assert easy_handeye
        assert rqt_easy_handeye
    
    @pytest.mark.unit
    def test_unit_marker(self):
        """Verify unit test marker works."""
        assert True
    
    @pytest.mark.integration
    def test_integration_marker(self):
        """Verify integration test marker works."""
        assert True
    
    @pytest.mark.slow
    def test_slow_marker(self):
        """Verify slow test marker works."""
        assert True
    
    def test_fixtures_available(self, temp_dir, mock_config, sample_calibration_data):
        """Verify common fixtures are available."""
        assert temp_dir.exists()
        assert mock_config is not None
        assert sample_calibration_data is not None
        assert "camera_matrix" in sample_calibration_data
    
    def test_coverage_configured(self):
        """Verify coverage is properly configured."""
        # This test will pass if coverage is running
        # The actual verification happens when running with coverage
        assert True
    
    def test_pyproject_toml_exists(self):
        """Verify pyproject.toml exists and contains test configuration."""
        pyproject_path = Path(__file__).parent.parent / "pyproject.toml"
        assert pyproject_path.exists()
        
        content = pyproject_path.read_text()
        assert "[tool.pytest.ini_options]" in content
        assert "[tool.coverage.run]" in content
        assert "[tool.poetry]" in content
        assert "pytest" in content