import requests
import time
import json
from typing import List, Dict, Optional, Tuple


class RobotController:
    """Контроллер для отправки команд на ESP32 робота"""

    def __init__(self, robot_ip: str = "192.168.1.100", port: int = 80):
        """
        Args:
            robot_ip: IP адрес робота в WiFi сети
            port: порт HTTP сервера (по умолчанию 80)
        """
        self.base_url = f"http://{robot_ip}:{port}"
        self.timeout = 30  # секунд

    def send_commands(self, commands: List[str]) -> Tuple[bool, str]:
        """
        Отправить список команд роботу

        Args:
            commands: список команд (строки)

        Returns:
            (success: bool, message: str)
        """
        # Формируем многострочный текст команд
        commands_text = "\n".join(commands)

        try:
            response = requests.post(
                f"{self.base_url}/command",
                data={"commands": commands_text},
                timeout=self.timeout
            )

            if response.status_code == 200:
                return True, response.text
            else:
                return False, f"HTTP {response.status_code}: {response.text}"

        except requests.exceptions.RequestException as e:
            return False, f"Ошибка соединения: {e}"

    def send_command(self, command: str) -> Tuple[bool, str]:
        """Отправить одну команду"""
        return self.send_commands([command])

    def go_to_point(self, x: float, y: float) -> Tuple[bool, str]:
        """
        Движение к точке в локальной системе координат

        Args:
            x: координата X (см)
            y: координата Y (см)
        """
        cmd = f"GO_LOCAL {x} {y}"
        return self.send_command(cmd)

    def lifter_up(self) -> Tuple[bool, str]:
        """Поднять подъёмник"""
        return self.send_command("UP")

    def lifter_down(self) -> Tuple[bool, str]:
        """Опустить подъёмник"""
        return self.send_command("DOWN")

    def delay(self, milliseconds: int) -> Tuple[bool, str]:
        """
        Задержка выполнения

        Args:
            milliseconds: время задержки в мс
        """
        return self.send_command(f"DELAY {milliseconds}")

    def get_status(self) -> Optional[Dict]:
        """
        Получить статус робота

        Returns:
            Словарь с телеметрией или None при ошибке
            {
                "x": float,
                "y": float,
                "angle": float,
                "lifter": "UP" или "DOWN",
                "history_size": int
            }
        """
        try:
            response = requests.get(
                f"{self.base_url}/status",
                timeout=5
            )

            if response.status_code == 200:
                return response.json()
            else:
                print(f"HTTP {response.status_code}: {response.text}")
                return None

        except requests.exceptions.RequestException as e:
            print(f"Ошибка соединения: {e}")
            return None

    def execute_route(self, route_points: List[Tuple[float, float]], 
                     lift_at_end: bool = False) -> Tuple[bool, str]:
        """
        Выполнить маршрут из нескольких точек

        Args:
            route_points: список точек [(x1, y1), (x2, y2), ...]
            lift_at_end: поднять подъёмник в конце маршрута

        Returns:
            (success: bool, message: str)
        """
        commands = []

        # Движение к каждой точке
        for i, (x, y) in enumerate(route_points):
            commands.append(f"GO_LOCAL {x} {y}")
            # Небольшая задержка между командами
            commands.append("DELAY 500")

        # Подъёмник в конце (если нужно)
        if lift_at_end:
            commands.append("UP")

        return self.send_commands(commands)

    def execute_graph_path(self, path: List[str], 
                          graph_data: Dict,
                          scale_x: float = 1.0,
                          scale_y: float = 1.0,
                          lift_at_nodes: Optional[List[str]] = None) -> Tuple[bool, str]:
        """
        Выполнить обход графа склада

        Args:
            path: список узлов графа ["0_0", "0_1", "0_2", ...]
            graph_data: данные графа с координатами узлов
            scale_x: масштаб по X (м/ед)
            scale_y: масштаб по Y (м/ед)
            lift_at_nodes: список узлов, где нужно поднять подъёмник

        Returns:
            (success: bool, message: str)
        """
        if lift_at_nodes is None:
            lift_at_nodes = []

        commands = []

        for node_id in path:
            # Получаем координаты узла из графа
            if 'nodes' in graph_data:
                node_data = None
                for node in graph_data['nodes']:
                    if node.get('id') == node_id:
                        node_data = node
                        break

                if node_data:
                    # Преобразуем координаты в метры
                    i = node_data.get('i', 0)
                    j = node_data.get('j', 0)
                    x = (i + 0.5) * scale_x * 100  # м → см
                    y = (j + 0.5) * scale_y * 100  # м → см

                    commands.append(f"GO_LOCAL {x:.1f} {y:.1f}")

                    # Подъёмник в нужных узлах
                    if node_id in lift_at_nodes:
                        commands.append("UP")
                        commands.append("DELAY 2000")  # Ждём 2 сек
                        commands.append("DOWN")

                    commands.append("DELAY 500")

        if not commands:
            return False, "Маршрут пуст"

        return self.send_commands(commands)


def send_route_to_robot(robot_ip: str, route: List[Dict], 
                       meta: Dict, axis_y: Optional[Dict] = None):
    """
    Отправить маршрут из веб-интерфейса на робота

    Args:
        robot_ip: IP адрес робота
        route: маршрут из интерфейса [{"i": 0, "j": 0, "id": "0_0"}, ...]
        meta: метаданные {"scaleX": 1.0, "scaleY": 1.0}
        axis_y: ось Y для преобразования координат
    """
    controller = RobotController(robot_ip)

    # Преобразуем клетки в метры
    scale_x = meta.get('scaleX', 1.0)
    scale_y = meta.get('scaleY', 1.0)

    commands = []

    for waypoint in route:
        i = waypoint.get('i', 0)
        j = waypoint.get('j', 0)

        # Преобразуем в метры (центр клетки)
        x_meters = (i + 0.5) * scale_x
        y_meters = (j + 0.5) * scale_y

        # Преобразуем в сантиметры для робота
        x_cm = x_meters * 100
        y_cm = y_meters * 100

        commands.append(f"GO_LOCAL {x_cm:.1f} {y_cm:.1f}")
        commands.append("DELAY 1000")  # 1 сек пауза между точками

    # Отправляем команды
    success, message = controller.send_commands(commands)

    if success:
        print(f"✅ Маршрут отправлен роботу ({len(route)} точек)")
    else:
        print(f"❌ Ошибка отправки: {message}")

    return success, message


# Пример использования
if __name__ == "__main__":
    # IP робота (указать актуальный)
    ROBOT_IP = "192.168.1.100"

    # Создаём контроллер
    controller = RobotController(ROBOT_IP)

    # Пример 1: Проверка статуса
    print("Проверка статуса робота...")
    status = controller.get_status()
    if status:
        print(f"Позиция: X={status['x']:.1f}, Y={status['y']:.1f}")
        print(f"Угол: {status['angle']:.1f}°")
        print(f"Подъёмник: {status['lifter']}")
    else:
        print("Робот недоступен")

    # Пример 2: Простой маршрут
    print("\nОтправка простого маршрута...")
    route_points = [
        (0, 0),      # Старт
        (50, 0),     # 50 см вперёд
        (50, 50),    # 50 см вправо
        (0, 0),      # Возврат
    ]

    # success, msg = controller.execute_route(route_points, lift_at_end=False)
    # print(f"Результат: {msg}")

    # Пример 3: Отправка одной команды
    print("\nОтправка команды GO_LOCAL...")
    # success, msg = controller.go_to_point(100, 100)
    # print(f"Результат: {msg}")

    print("\nГотово!")
