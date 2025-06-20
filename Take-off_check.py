# Проверка перед взлётом
def preflight_check():
    assert drone.get_imu_status(), "IMU не готов"
    assert lidar.is_connected(), "LiDAR не отвечает"
    return True

if preflight_check():
    drone.takeoff()