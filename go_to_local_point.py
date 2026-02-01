from pioneer_sdk import Pioneer
import time

# Инициализация дрона
pioneer = Pioneer()

try:
    # Взлет на 1 метр
    pioneer.arm()
    pioneer.takeoff()
    pioneer.go_to_local_point(x=0, y=0, z=2, yaw=0)
    while not pioneer.point_reached():
        time.sleep(0.1)

    # Полет в первую точку
    pioneer.go_to_local_point(x=0, y=0.5, z=2, yaw=0)
    while not pioneer.point_reached():
        time.sleep(0.1)
    time.sleep(5)
    # Полет во вторую точку
    pioneer.go_to_local_point(x=0, y=1, z=2, yaw=0)
    while not pioneer.point_reached():
        time.sleep(0.1)
    time.sleep(5)
    pioneer.go_to_local_point(x=0.5, y=0.5, z=2, yaw=0)
    while not pioneer.point_reached():
        time.sleep(0.1)
    time.sleep(5)

    pioneer.go_to_local_point(x=0, y=0, z=2, yaw=0)
    while not pioneer.point_reached():
        time.sleep(0.1)
    

    pioneer.land()

except KeyboardInterrupt:
    print("Остановка программы, производится посадка")
    pioneer.land()
    
