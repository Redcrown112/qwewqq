assert lidar.ping(), "LiDAR не отвечает"
assert drone.get_signal_strength() > 80, "Слабый сигнал"