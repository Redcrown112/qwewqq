# По SLAM-карте
drone.return_to_home()

# Контроль процесса
while drone.is_moving():
    if drone.get_battery() < 20:
        drone.emergency_land()