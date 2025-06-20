# Запуск SLAM (LIO-SAM через ROS)
os.system("roslaunch lio_sam run.launch pointCloudMinRange:=0.5 &")

# Взлёт на высоту 3 м (в локальных координатах)
drone.takeoff()
drone.go_to_local_position(x=0, y=0, z=-3, speed=0.5)  # Ось Z вниз!

# Автономный облёт с динамическим избеганием
drone.start_exploration_mode(
    max_speed=1.5,  # м/с
    exploration_radius=30,  # макс. радиус от старта
    min_altitude=-20,  # глубина шахты
    obstacle_threshold=1.0  # м
)

# Мониторинг статуса
while drone.is_exploring():
    if drone.get_battery() < 25:
        drone.return_to_home()