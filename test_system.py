#!/usr/bin/env python3

"""
WARG Drone Competition 2025 - System Test Script
Basic validation of the modular components
"""

import sys
import logging

def test_imports():
    """Test if all modules can be imported successfully"""
    print("Testing module imports...")
    
    try:
        import config
        print("✓ Config module imported successfully")
    except ImportError as e:
        print(f"✗ Config module failed: {e}")
        return False
    
    try:
        from building_mapper import BuildingMapper
        print("✓ BuildingMapper imported successfully")
    except ImportError as e:
        print(f"✗ BuildingMapper failed: {e}")
        return False
    
    try:
        from target_detector import TargetDetector
        print("✓ TargetDetector imported successfully")
    except ImportError as e:
        print(f"✗ TargetDetector failed: {e}")
        return False
    
    try:
        from camera_manager import CameraManager
        print("✓ CameraManager imported successfully")
    except ImportError as e:
        print(f"✗ CameraManager failed: {e}")
        return False
    
    try:
        import utils
        print("✓ Utils module imported successfully")
    except ImportError as e:
        print(f"✗ Utils module failed: {e}")
        return False
    
    return True

def test_building_mapper():
    """Test basic building mapper functionality"""
    print("\nTesting BuildingMapper...")
    
    try:
        from building_mapper import BuildingMapper
        
        mapper = BuildingMapper()
        
        # Test adding corners
        test_corners = [
            (43.4725, -80.5448),  # NW
            (43.4725, -80.5440),  # NE
            (43.4720, -80.5440),  # SE
            (43.4720, -80.5448)   # SW
        ]
        
        for i, (lat, lon) in enumerate(test_corners):
            success = mapper.add_corner(lat, lon)
            if not success:
                print(f"✗ Failed to add corner {i+1}")
                return False
        
        # Test building info
        info = mapper.get_building_info()
        if info['corner_count'] == 4 and info['is_complete']:
            print("✓ BuildingMapper basic functionality works")
            print(f"  - Building: {info['dimensions']['width']:.1f}m x {info['dimensions']['height']:.1f}m")
            return True
        else:
            print("✗ BuildingMapper failed to process 4 corners")
            return False
            
    except Exception as e:
        print(f"✗ BuildingMapper test failed: {e}")
        return False

def test_target_detector():
    """Test target detector creation"""
    print("\nTesting TargetDetector...")
    
    try:
        from target_detector import TargetDetector
        import numpy as np
        
        detector = TargetDetector()
        
        # Create a dummy frame
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Test detection (should return empty list for black frame)
        targets = detector.detect_targets(dummy_frame)
        
        if isinstance(targets, list):
            print("✓ TargetDetector basic functionality works")
            print(f"  - Detected {len(targets)} targets in test frame")
            return True
        else:
            print("✗ TargetDetector did not return a list")
            return False
            
    except Exception as e:
        print(f"✗ TargetDetector test failed: {e}")
        return False

def test_utils():
    """Test utility functions"""
    print("\nTesting Utils...")
    
    try:
        from utils import haversine_distance, GPS, calculate_bearing
        
        # Test GPS distance calculation
        gps1 = GPS(43.4725, -80.5448)
        gps2 = GPS(43.4720, -80.5440)
        
        distance = haversine_distance(gps1.lat, gps1.lon, gps2.lat, gps2.lon)
        
        if 50 < distance < 150:  # Should be around 80-100 meters
            print("✓ Utils functions work correctly")
            print(f"  - Distance calculation: {distance:.1f}m")
            return True
        else:
            print(f"✗ Distance calculation seems wrong: {distance:.1f}m")
            return False
            
    except Exception as e:
        print(f"✗ Utils test failed: {e}")
        return False

def test_config():
    """Test configuration values"""
    print("\nTesting Config...")
    
    try:
        from config import TARGET_COLORS, LOCKED_RADIUS, RC_THRESHOLD
        
        if len(TARGET_COLORS) == 6:  # Should have 6 colors + red_upper
            print("✓ Config values loaded correctly")
            print(f"  - Target colors: {len(TARGET_COLORS)} defined")
            print(f"  - Lock radius: {LOCKED_RADIUS}px")
            return True
        else:
            print(f"✗ Expected 6+ target colors, got {len(TARGET_COLORS)}")
            return False
            
    except Exception as e:
        print(f"✗ Config test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("WARG Drone Competition 2025 - System Validation")
    print("=" * 50)
    
    # Suppress logging for cleaner test output
    logging.getLogger().setLevel(logging.WARNING)
    
    tests = [
        test_imports,
        test_config,
        test_utils,
        test_building_mapper,
        test_target_detector
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"✗ Test {test.__name__} crashed: {e}")
    
    print("\n" + "=" * 50)
    print(f"Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All tests passed! System ready for competition.")
        return 0
    else:
        print(f"⚠️  {total - passed} test(s) failed. Please check the issues above.")
        return 1

if __name__ == "__main__":
    exit(main())