# Инициализация (DJI SDK)
drone = DJIMatrice350()
drone.set_flight_mode("ATTITUDE")  # Режим без GPS
drone.set_min_altitude(-50)  # Отрицательная высота для шахты
drone.enable_low_battery_return(True)  # Автовозврат при 20% заряда