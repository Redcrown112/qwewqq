# Принудительная посадка при ошибках
except Exception as e:
    drone.emergency_land()
    log_error(f"Авария: {str(e)}")