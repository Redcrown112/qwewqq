# Взлёт и стабилизация
drone.takeoff()
time.sleep(5)  # Ожидание стабилизации IMU

# Проверка SLAM перед стартом
if not slam_odometry.is_ready():
    drone.land()
    raise Exception("SLAM не инициализирован!")

# Загрузка и выполнение маршрута
drone.upload_mission(waypoints)
drone.start_mission()

# Контроль выполнения
for wp in waypoints:
    drone.move_to_waypoint(wp["id"]) 
    while drone.get_distance_to_target() > 0.5:  # Точность 0.5 м
        if slam_odometry.confidence < 0.2:
            drone.hold_position()  # Стоп при потере трека
            time.sleep(1)