"""
Пример отправки маршрута на робота ESP32
Показывает полный цикл: генерация склада → построение маршрута → отправка роботу
"""

import sys
import os

# Добавляем родительскую директорию для импорта
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from deykstra import WarehouseGenerator, WarehouseNavigator
from service.robot_controller import RobotController, send_route_to_robot


def generate_and_send_route(robot_ip: str = "192.168.1.100"):
    """
    Генерирует склад, строит маршрут и отправляет роботу

    Args:
        robot_ip: IP адрес ESP32 робота
    """
    print("=" * 70)
    print("ГЕНЕРАЦИЯ МАРШРУТА И ОТПРАВКА РОБОТУ")
    print("=" * 70)

    # 1. Генерация склада
    print("\n1️⃣  Генерация склада...")
    generator = WarehouseGenerator(
        rows=6,
        cols=8,
        aisle_prob=0.25,
        seed=42
    )

    grid, walkable = generator.generate_warehouse()
    graph = generator.grid_to_graph(grid, walkable)

    print(f"   Размер: {len(grid)}x{len(grid[0])}")
    print(f"   Узлов в графе: {len(graph.nodes())}")

    # 2. Создание навигатора и построение маршрута
    print("\n2️⃣  Построение маршрута...")
    navigator = WarehouseNavigator(graph, grid)

    # Выбираем стартовую точку
    start_node = list(graph.nodes())[0]
    print(f"   Стартовая точка: {start_node}")

    # Используем оптимальный алгоритм обхода
    path = navigator.warehouse_optimal_traversal(start_node)
    print(f"   Длина маршрута: {len(path)} точек")

    # 3. Преобразование маршрута в формат для робота
    print("\n3️⃣  Преобразование маршрута...")
    route = []
    for i, node_id in enumerate(path):
        r, c = map(int, node_id.split('_'))
        route.append({
            'id': node_id,
            'i': c,  # колонка
            'j': r   # ряд
        })

    # Метаданные (масштаб: 1 клетка = 1 метр)
    meta = {
        'scaleX': 1.0,  # 1 метр по X
        'scaleY': 1.0   # 1 метр по Y
    }

    print(f"   Точек в маршруте: {len(route)}")
    print(f"   Масштаб: {meta['scaleX']}м x {meta['scaleY']}м")

    # 4. Отправка роботу
    print("\n4️⃣  Отправка маршрута роботу...")
    print(f"   IP робота: {robot_ip}")

    # Раскомментируйте для реальной отправки:
    # success, message = send_route_to_robot(robot_ip, route, meta)
    #
    # if success:
    #     print(f"   ✅ Маршрут успешно отправлен!")
    # else:
    #     print(f"   ❌ Ошибка: {message}")

    print("\n   ⚠️  Для реальной отправки раскомментируйте код в конце скрипта")

    # 5. Визуализация (опционально)
    print("\n5️⃣  Визуализация маршрута...")
    try:
        from deykstra import visualize_warehouse
        visualize_warehouse(
            grid, graph, path, start_node,
            "Маршрут для робота"
        )
    except Exception as e:
        print(f"   Визуализация пропущена: {e}")

    print("\n" + "=" * 70)
    print("ГОТОВО!")
    print("=" * 70)

    return route, meta


def send_simple_test_route(robot_ip: str):
    """
    Отправить простой тестовый маршрут роботу
    """
    print("\n" + "=" * 70)
    print("ТЕСТОВАЯ ОТПРАВКА МАРШРУТА РОБОТУ")
    print("=" * 70)

    controller = RobotController(robot_ip)

    # Проверяем соединение
    print("\n📡 Проверка соединения с роботом...")
    status = controller.get_status()

    if status:
        print(f"✅ Робот доступен")
        print(f"   Позиция: X={status['x']:.1f}, Y={status['y']:.1f}")
        print(f"   Угол: {status['angle']:.1f}°")
        print(f"   Подъёмник: {status['lifter']}")
    else:
        print(f"❌ Робот недоступен. Проверьте IP адрес и WiFi соединение.")
        return

    # Простой маршрут (в метрах, робот конвертирует в см)
    print("\n📍 Отправка тестового маршрута...")
    route = [
        {'i': 0, 'j': 0, 'id': '0_0'},
        {'i': 1, 'j': 0, 'id': '0_1'},
        {'i': 2, 'j': 0, 'id': '0_2'},
        {'i': 2, 'j': 1, 'id': '1_2'},
        {'i': 1, 'j': 1, 'id': '1_1'},
        {'i': 0, 'j': 0, 'id': '0_0'},  # Возврат
    ]

    meta = {'scaleX': 1.0, 'scaleY': 1.0}

    # Раскомментируйте для отправки:
    # success, message = send_route_to_robot(robot_ip, route, meta)
    #
    # if success:
    #     print(f"✅ Маршрут отправлен!")
    # else:
    #     print(f"❌ Ошибка: {message}")

    print("\n   ⚠️  Раскомментируйте код для реальной отправки")


def send_manual_commands(robot_ip: str):
    """
    Отправить команды роботу вручную
    """
    print("\n" + "=" * 70)
    print("РУЧНАЯ ОТПРАВКА КОМАНД РОБОТУ")
    print("=" * 70)

    controller = RobotController(robot_ip)

    # Пример отправки отдельных команд
    print("\n📤 Отправка команд...")

    # 1. Проверка статуса
    print("\n1. Запрос статуса...")
    status = controller.get_status()
    if status:
        print(f"   Статус: {status}")

    # 2. Движение к точке (50 см, 0 см)
    print("\n2. Движение к точке (50см, 0см)...")
    # success, msg = controller.go_to_point(50, 0)
    # print(f"   Результат: {msg}")

    # 3. Поднять подъёмник
    print("\n3. Подъём подъёмника...")
    # success, msg = controller.lifter_up()
    # print(f"   Результат: {msg}")

    # 4. Задержка
    print("\n4. Задержка 2 секунды...")
    # success, msg = controller.delay(2000)
    # print(f"   Результат: {msg}")

    # 5. Опустить подъёмник
    print("\n5. Опускание подъёмника...")
    # success, msg = controller.lifter_down()
    # print(f"   Результат: {msg}")

    print("\n   ⚠️  Раскомментируйте код для реальной отправки")


if __name__ == "__main__":
    # IP адрес робота (измените на актуальный!)
    ROBOT_IP = "192.168.1.100"

    # Вариант 1: Полный цикл (генерация → маршрут → отправка)
    # route, meta = generate_and_send_route(ROBOT_IP)

    # Вариант 2: Простой тестовый маршрут
    # send_simple_test_route(ROBOT_IP)

    # Вариант 3: Ручные команды
    # send_manual_commands(ROBOT_IP)

    print("\n📋 Выберите вариант:")
    print("   1. Полный цикл (генерация склада + маршрут + отправка)")
    print("   2. Простой тестовый маршрут")
    print("   3. Ручные команды")
    print("\n   Раскомментируйте нужный вариант в коде выше")
