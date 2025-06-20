# Остановка записи LiDAR
livox_lidar_console --cmd stop_logging

# Архивирование данных
tar -czvf scan_data.tar.gz scan_*.lvx slam_data.bag