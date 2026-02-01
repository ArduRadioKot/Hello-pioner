import heapq
import random
import matplotlib.pyplot as plt
import networkx as nx
from typing import List, Tuple, Dict, Set
import numpy as np
from collections import deque

class WarehouseGenerator:
    """Генератор случайного склада в форме четырёхугольника"""
    
    def __init__(self, rows: int = 6, cols: int = 8, 
                 aisle_prob: float = 0.25, seed: int = None):
        """
        Args:
            rows: количество рядов стеллажей
            cols: количество колонок стеллажей
            aisle_prob: вероятность прохода между стеллажами (кроме обязательных)
            seed: seed для воспроизводимости
        """
        self.rows = rows
        self.cols = cols
        self.aisle_prob = aisle_prob
        if seed:
            random.seed(seed)
            np.random.seed(seed)
        
        # Обязательные проходы: по краям и через каждые два стеллажа
        self.mandatory_aisles = self._get_mandatory_aisles()
    
    def _get_mandatory_aisles(self) -> Set[Tuple[int, int]]:
        """Получить координаты обязательных проходов"""
        aisles = set()
        
        # Проходы по краям (периметр склада)
        for r in range(self.rows):
            aisles.add((r, 0))  # Левый край
            aisles.add((r, self.cols - 1))  # Правый край
        
        # Проход через каждые два стеллажа (через каждые 3 колонки)
        for c in range(2, self.cols - 1, 3):
            for r in range(self.rows):
                aisles.add((r, c))
        
        # Проходы сверху и снизу
        for c in range(self.cols):
            aisles.add((0, c))  # Верхний край
            aisles.add((self.rows - 1, c))  # Нижний край
        
        return aisles
    
    def generate_warehouse(self) -> Tuple[List[List[str]], List[List[bool]]]:
        """
        Генерация склада
        
        Returns:
            grid: сетка склада (S - стеллаж, . - проход)
            walkable: матрица проходимости (True - можно пройти)
        """
        # Создаем сетку - все клетки изначально стеллажи
        grid = [['S' for _ in range(self.cols)] for _ in range(self.rows)]
        walkable = [[False for _ in range(self.cols)] for _ in range(self.rows)]
        
        # Проставляем обязательные проходы
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) in self.mandatory_aisles:
                    grid[r][c] = '.'
                    walkable[r][c] = True
        
        # Добавляем случайные проходы ТОЛЬКО МЕЖДУ стеллажами
        for r in range(1, self.rows - 1):  # Не по краям
            for c in range(1, self.cols - 1):  # Не по краям
                # Проверяем, что это стеллаж И не обязательный проход
                if grid[r][c] == 'S' and (r, c) not in self.mandatory_aisles:
                    # Проверяем соседей: чтобы был хотя бы один проход рядом
                    has_nearby_aisle = False
                    # ТОЛЬКО 4 направления (без диагоналей!)
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.rows and 0 <= nc < self.cols:
                            if walkable[nr][nc]:
                                has_nearby_aisle = True
                                break
                    
                    if has_nearby_aisle and random.random() < self.aisle_prob:
                        grid[r][c] = '.'
                        walkable[r][c] = True
        
        # Убедимся, что все проходы соединены
        self._ensure_connectivity(grid, walkable)
        
        return grid, walkable
    
    def _ensure_connectivity(self, grid: List[List[str]], walkable: List[List[bool]]):
        """Убедиться, что все проходы связаны (можно дойти от любого к любому)"""
        # Находим все проходы
        passages = []
        for r in range(self.rows):
            for c in range(self.cols):
                if walkable[r][c]:
                    passages.append((r, c))
        
        if not passages:
            return
        
        # Проверяем связность с помощью BFS
        start = passages[0]
        visited = set()
        queue = deque([start])
        
        while queue:
            r, c = queue.popleft()
            if (r, c) in visited:
                continue
            visited.add((r, c))
            
            # ТОЛЬКО 4 направления (без диагоналей!)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if walkable[nr][nc] and (nr, nc) not in visited:
                        queue.append((nr, nc))
        
        # Если не все проходы связаны, добавляем соединения
        if len(visited) < len(passages):
            # Находим несвязанные проходы
            unconnected = [p for p in passages if p not in visited]
            
            # Связываем ближайшие проходы
            for uc_r, uc_c in unconnected:
                # Ищем ближайший связанный проход
                min_dist = float('inf')
                nearest = None
                
                for v_r, v_c in visited:
                    # Манхэттенское расстояние (только по осям)
                    dist = abs(uc_r - v_r) + abs(uc_c - v_c)
                    if dist < min_dist:
                        min_dist = dist
                        nearest = (v_r, v_c)
                
                # Создаем путь между ними (только по осям)
                if nearest:
                    self._create_path_between(grid, walkable, (uc_r, uc_c), nearest)
    
    def _create_path_between(self, grid: List[List[str]], walkable: List[List[bool]], 
                            start: Tuple[int, int], end: Tuple[int, int]):
        """Создает проход между двумя точками (только по осям!)"""
        r1, c1 = start
        r2, c2 = end
        
        # Создаем путь L-образно: сначала горизонтально, потом вертикально
        r, c = r1, c1
        
        # Горизонтальное движение
        step_c = 1 if c2 > c else -1
        while c != c2:
            # Проверяем, не выходим ли за границы
            if 0 <= r < self.rows and 0 <= c < self.cols:
                grid[r][c] = '.'
                walkable[r][c] = True
            c += step_c
        
        # Вертикальное движение
        step_r = 1 if r2 > r else -1
        while r != r2:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                grid[r][c] = '.'
                walkable[r][c] = True
            r += step_r
    
    def grid_to_graph(self, grid: List[List[str]], 
                     walkable: List[List[bool]]) -> nx.Graph:
        """Преобразование сетки в граф (ТОЛЬКО проходимые клетки, БЕЗ диагоналей)"""
        G = nx.Graph()
        
        # Добавляем вершины ТОЛЬКО для проходимых клеток
        for r in range(self.rows):
            for c in range(self.cols):
                if walkable[r][c]:
                    node_id = f"{r}_{c}"
                    G.add_node(node_id, pos=(c, -r), type=grid[r][c])
        
        # Добавляем рёбра ТОЛЬКО между соседними проходимыми клетками
        # Рёбра только в 4 направлениях (без диагоналей!)
        for r in range(self.rows):
            for c in range(self.cols):
                if walkable[r][c]:
                    node_id = f"{r}_{c}"
                    
                    # ТОЛЬКО 4 соседа (вверх, вниз, влево, вправо)
                    # НЕТ диагональных перемещений!
                    neighbors = [
                        (r-1, c),  # вверх
                        (r+1, c),  # вниз
                        (r, c-1),  # влево
                        (r, c+1)   # вправо
                    ]
                    
                    for nr, nc in neighbors:
                        if 0 <= nr < self.rows and 0 <= nc < self.cols:
                            if walkable[nr][nc]:  # ТОЛЬКО если сосед проходим
                                neighbor_id = f"{nr}_{nc}"
                                # Вес 1 для всех рёбер
                                if not G.has_edge(node_id, neighbor_id):
                                    G.add_edge(node_id, neighbor_id, weight=1)
        
        return G

class WarehouseNavigator:
    """Навигатор по складу с алгоритмами обхода"""
    
    def __init__(self, graph: nx.Graph, grid: List[List[str]]):
        self.graph = graph
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.node_to_coords = self._create_coord_mapping()
    
    def _create_coord_mapping(self) -> Dict[str, Tuple[int, int]]:
        """Создает отображение ID узла в координаты"""
        mapping = {}
        for node in self.graph.nodes():
            r_str, c_str = node.split('_')
            mapping[node] = (int(r_str), int(c_str))
        return mapping
    
    def bfs_traversal(self, start_node: str) -> List[str]:
        """Обход в ширину (BFS) - находит кратчайшие пути"""
        if start_node not in self.graph:
            raise ValueError(f"Стартовая вершина {start_node} не существует")
        
        visited = set([start_node])
        queue = deque([start_node])
        traversal_order = [start_node]
        
        while queue:
            current = queue.popleft()
            
            # Перебираем всех соседей (всего 4 возможных направления)
            for neighbor in sorted(self.graph.neighbors(current)):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    traversal_order.append(neighbor)
        
        return traversal_order
    
    def dfs_traversal(self, start_node: str) -> List[str]:
        """Обход в глубину (DFS)"""
        if start_node not in self.graph:
            raise ValueError(f"Стартовая вершина {start_node} не существует")
        
        visited = set()
        traversal_order = []
        
        def dfs(node):
            visited.add(node)
            traversal_order.append(node)
            
            # Перебираем соседей в порядке: вправо, вниз, влево, вверх
            # (эвристика для лучшего покрытия склада)
            neighbors = list(self.graph.neighbors(node))
            # Сортируем по координатам для более систематичного обхода
            neighbors.sort(key=lambda n: (self.node_to_coords[n][0], 
                                         self.node_to_coords[n][1]))
            
            for neighbor in neighbors:
                if neighbor not in visited:
                    dfs(neighbor)
        
        dfs(start_node)
        return traversal_order
    
    def nearest_neighbor_tsp(self, start_node: str) -> List[str]:
        """Алгоритм ближайшего соседа (жадный алгоритм)"""
        if start_node not in self.graph:
            raise ValueError(f"Стартовая вершина {start_node} не существует")
        
        unvisited = set(self.graph.nodes())
        unvisited.remove(start_node)
        
        path = [start_node]
        current = start_node
        
        while unvisited:
            # Находим ближайшего непосещённого соседа
            nearest = None
            min_dist = float('inf')
            
            # Сначала проверяем непосредственных соседей (расстояние 1)
            for neighbor in self.graph.neighbors(current):
                if neighbor in unvisited:
                    dist = 1  # Все непосредственные соседи на расстоянии 1
                    if dist < min_dist:
                        min_dist = dist
                        nearest = neighbor
            
            if nearest is None:
                # Ищем любой непосещённый узел через кратчайший путь
                for node in unvisited:
                    try:
                        # Используем BFS для нахождения расстояния
                        dist = nx.shortest_path_length(self.graph, current, node)
                        if dist < min_dist:
                            min_dist = dist
                            nearest = node
                    except nx.NetworkXNoPath:
                        continue
            
            if nearest is None:
                break  # Нет достижимых вершин
            
            # Добавляем путь до ближайшего соседа
            try:
                shortest_path = nx.shortest_path(self.graph, current, nearest)[1:]
                path.extend(shortest_path)
                current = nearest
                unvisited.remove(nearest)
            except nx.NetworkXNoPath:
                break
        
        return path
    
    def warehouse_optimal_traversal(self, start_node: str) -> List[str]:
        """
        Оптимальный обход склада с минимизацией повторных посещений
        Использует стратегию "змейка" по проходам
        """
        if start_node not in self.graph:
            raise ValueError(f"Стартовая вершина {start_node} не существует")
        
        # Получаем все узлы графа
        all_nodes = set(self.graph.nodes())
        unvisited = all_nodes.copy()
        unvisited.remove(start_node)
        
        path = [start_node]
        current = start_node
        current_r, current_c = self.node_to_coords[start_node]
        
        # Определяем основные направления движения
        # Предпочитаем движение по рядам
        direction = 'right'  # Начинаем движение вправо
        
        while unvisited:
            moved = False
            
            # Пробуем двигаться в текущем направлении
            if direction == 'right':
                next_node = f"{current_r}_{current_c + 1}"
                if next_node in self.graph and next_node in unvisited:
                    path.append(next_node)
                    current = next_node
                    current_c += 1
                    unvisited.remove(next_node)
                    moved = True
                else:
                    # Меняем направление
                    direction = 'down_right'
            
            elif direction == 'left':
                next_node = f"{current_r}_{current_c - 1}"
                if next_node in self.graph and next_node in unvisited:
                    path.append(next_node)
                    current = next_node
                    current_c -= 1
                    unvisited.remove(next_node)
                    moved = True
                else:
                    # Меняем направление
                    direction = 'down_left'
            
            elif direction == 'down_right':
                # Спускаемся на ряд ниже и меняем направление на left
                next_node = f"{current_r + 1}_{current_c}"
                if next_node in self.graph and next_node in unvisited:
                    path.append(next_node)
                    current = next_node
                    current_r += 1
                    unvisited.remove(next_node)
                    direction = 'left'
                    moved = True
                else:
                    # Пробуем найти любой доступный узел
                    direction = 'nearest'
            
            elif direction == 'down_left':
                # Спускаемся на ряд ниже и меняем направление на right
                next_node = f"{current_r + 1}_{current_c}"
                if next_node in self.graph and next_node in unvisited:
                    path.append(next_node)
                    current = next_node
                    current_r += 1
                    unvisited.remove(next_node)
                    direction = 'right'
                    moved = True
                else:
                    # Пробуем найти любой доступный узел
                    direction = 'nearest'
            
            if not moved:
                # Ищем ближайший непосещённый узел
                nearest = self._find_nearest_unvisited(current, unvisited)
                if nearest is None:
                    break
                
                # Идем к ближайшему непосещённому
                try:
                    shortest_path = nx.shortest_path(self.graph, current, nearest)[1:]
                    path.extend(shortest_path)
                    
                    # Обновляем текущую позицию
                    if shortest_path:
                        current = shortest_path[-1]
                        current_r, current_c = self.node_to_coords[current]
                        unvisited.remove(current)
                    
                    # Определяем новое направление
                    if len(shortest_path) > 0:
                        # Определяем общее направление движения
                        last_node = shortest_path[-1]
                        last_r, last_c = self.node_to_coords[last_node]
                        
                        if last_c > current_c:
                            direction = 'right'
                        elif last_c < current_c:
                            direction = 'left'
                        else:
                            # Двигались вертикально, сохраняем предыдущее направление
                            pass
                    
                except nx.NetworkXNoPath:
                    break
        
        return path
    
    def _find_nearest_unvisited(self, start: str, unvisited: Set[str]) -> str:
        """Находит ближайший непосещённый узел"""
        nearest = None
        min_dist = float('inf')
        
        for node in unvisited:
            try:
                # Используем BFS для нахождения расстояния
                dist = nx.shortest_path_length(self.graph, start, node)
                if dist < min_dist:
                    min_dist = dist
                    nearest = node
            except nx.NetworkXNoPath:
                continue
        
        return nearest
    
    def calculate_path_metrics(self, path: List[str]) -> Dict:
        """Вычисление метрик пути"""
        if not path:
            return {
                "length": 0, 
                "unique_nodes": 0, 
                "repeats": 0, 
                "coverage": 0.0,
                "efficiency": 0.0
            }
        
        total_length = len(path) - 1  # Количество шагов
        unique_nodes = len(set(path))
        repeats = total_length + 1 - unique_nodes  # Повторные посещения
        total_nodes = len(self.graph.nodes())
        coverage = unique_nodes / total_nodes if total_nodes > 0 else 0.0
        efficiency = unique_nodes / total_length if total_length > 0 else 0.0
        
        return {
            "length": total_length,
            "unique_nodes": unique_nodes,
            "repeats": repeats,
            "coverage": coverage,
            "efficiency": efficiency
        }

def visualize_warehouse(grid: List[List[str]], 
                       graph: nx.Graph,
                       path: List[str] = None,
                       start_node: str = None,
                       title: str = "План склада"):
    """Визуализация склада и пути обхода"""
    fig, axes = plt.subplots(1, 3 if path else 2, figsize=(18, 6))
    
    if path:
        ax1, ax2, ax3 = axes
    else:
        ax1, ax2 = axes
    
    rows = len(grid)
    cols = len(grid[0])
    
    # 1. Визуализация сетки склада
    ax1.set_title("Сетка склада", fontsize=14, fontweight='bold')
    ax1.set_aspect('equal')
    
    # Рисуем сетку
    for r in range(rows + 1):
        ax1.axhline(y=-r, xmin=0, xmax=cols, color='black', linewidth=0.5)
    for c in range(cols + 1):
        ax1.axvline(x=c, ymin=0, ymax=-rows, color='black', linewidth=0.5)
    
    # Закрашиваем клетки
    for r in range(rows):
        for c in range(cols):
            cell_type = grid[r][c]
            if cell_type == 'S':
                # Стеллаж - темно-серый
                color = '#666666'
                ax1.add_patch(plt.Rectangle((c, -r-1), 1, 1, 
                                          facecolor=color, edgecolor='black'))
                ax1.text(c + 0.5, -r - 0.5, 'S', 
                        ha='center', va='center', fontsize=10,
                        fontweight='bold', color='white')
            else:
                # Проход - светло-голубой
                color = '#e6f7ff'
                ax1.add_patch(plt.Rectangle((c, -r-1), 1, 1, 
                                          facecolor=color, edgecolor='black'))
                ax1.text(c + 0.5, -r - 0.5, '.', 
                        ha='center', va='center', fontsize=14, color='blue')
    
    ax1.set_xlim(-0.5, cols + 0.5)
    ax1.set_ylim(-rows - 0.5, 0.5)
    ax1.set_xticks(range(cols))
    ax1.set_yticks([-i for i in range(rows)])
    ax1.set_xticklabels(range(cols))
    ax1.set_yticklabels(range(rows))
    ax1.set_xlabel("Колонки")
    ax1.set_ylabel("Строки")
    ax1.grid(True, alpha=0.3)
    
    # 2. Визуализация графа
    ax2.set_title("Граф проходов (движение только по осям)", 
                 fontsize=14, fontweight='bold')
    
    # Позиции вершин
    pos = nx.get_node_attributes(graph, 'pos')
    
    # Рисуем граф
    nx.draw_networkx_nodes(graph, pos, ax=ax2, 
                          node_color='lightblue', 
                          node_size=200,
                          node_shape='s',
                          edgecolors='black')
    nx.draw_networkx_edges(graph, pos, ax=ax2, 
                          edge_color='gray', 
                          width=1.5, 
                          alpha=0.7,
                          style='solid',  # Сплошные линии (не диагонали)
                          connectionstyle='arc3,rad=0')  # Прямые линии
    
    # Подписываем только некоторые узлы для читаемости
    if len(graph.nodes()) <= 50:
        nx.draw_networkx_labels(graph, pos, ax=ax2, font_size=8)
    
    # Выделяем путь, если он задан
    if path and len(path) > 1:
        # Рисуем рёбра пути
        path_edges = list(zip(path[:-1], path[1:]))
        nx.draw_networkx_edges(graph, pos, ax=ax2, 
                              edgelist=path_edges,
                              edge_color='red', 
                              width=3,
                              alpha=0.8,
                              style='solid',
                              connectionstyle='arc3,rad=0')
        
        # Выделяем вершины пути
        path_nodes = set(path)
        nx.draw_networkx_nodes(graph, pos, ax=ax2, 
                              nodelist=list(path_nodes),
                              node_color='red', 
                              node_size=250,
                              node_shape='s', 
                              alpha=0.7,
                              edgecolors='darkred')
        
        # Выделяем стартовую вершину
        if start_node:
            nx.draw_networkx_nodes(graph, pos, ax=ax2, 
                                  nodelist=[start_node],
                                  node_color='green', 
                                  node_size=300,
                                  node_shape='s',
                                  edgecolors='darkgreen')
    
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    ax2.set_xlabel("Движение: ↑ ↓ ← → (диагонали запрещены)")
    
    # 3. Визуализация покрытия пути (если есть путь)
    if path:
        ax3.set_title(f"Покрытие пути: {title}", fontsize=14, fontweight='bold')
        ax3.set_aspect('equal')
        
        # Создаем матрицу покрытия
        coverage_grid = [[' ' for _ in range(cols)] for _ in range(rows)]
        visited_nodes = set(path)
        
        # Заполняем матрицу символами
        for r in range(rows):
            for c in range(cols):
                node_id = f"{r}_{c}"
                if grid[r][c] == 'S':
                    coverage_grid[r][c] = '█'  # Стеллаж
                elif node_id in graph.nodes():
                    if node_id in visited_nodes:
                        coverage_grid[r][c] = '●'  # Посещено
                    else:
                        coverage_grid[r][c] = '○'  # Доступно, но не посещено
                elif grid[r][c] == '.':
                    coverage_grid[r][c] = '·'  # Пустой проход
        
        # Отображаем текстовую сетку
        ax3.set_xlim(-0.5, cols - 0.5)
        ax3.set_ylim(-rows + 0.5, 0.5)
        
        # Отключаем оси
        ax3.set_xticks([])
        ax3.set_yticks([])
        
        # Отображаем символы
        for r in range(rows):
            for c in range(cols):
                symbol = coverage_grid[r][c]
                color = 'black'
                
                if symbol == '█':  # Стеллаж
                    color = 'gray'
                    fontsize = 12
                elif symbol == '●':  # Посещено
                    color = 'red'
                    fontsize = 14
                elif symbol == '○':  # Доступно
                    color = 'blue'
                    fontsize = 12
                elif symbol == '·':  # Проход
                    color = 'lightgray'
                    fontsize = 10
                
                ax3.text(c, -r, symbol, 
                        ha='center', va='center',
                        fontsize=fontsize, color=color,
                        fontfamily='monospace')
        
        # Добавляем сетку
        for r in range(rows + 1):
            ax3.axhline(y=-r + 0.5, xmin=0, xmax=cols, 
                       color='gray', linewidth=0.5, alpha=0.5)
        for c in range(cols + 1):
            ax3.axvline(x=c - 0.5, ymin=0, ymax=-rows, 
                       color='gray', linewidth=0.5, alpha=0.5)
        
        # Вычисляем и отображаем метрики
        total_accessible = len(graph.nodes())
        total_visited = len(visited_nodes)
        coverage_pct = (total_visited / total_accessible * 100) if total_accessible > 0 else 0
        
        info_text = f"Покрыто: {total_visited}/{total_accessible} узлов\n"
        info_text += f"({coverage_pct:.1f}% доступной зоны)"
        
        ax3.text(0.5, 0.98, info_text,
                transform=ax3.transAxes,
                ha='center', va='top',
                fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.9))
    
    plt.tight_layout()
    plt.show()

def compare_algorithms(warehouse: WarehouseNavigator, start_node: str):
    """Сравнение различных алгоритмов обхода"""
    algorithms = {
        "BFS (поиск в ширину)": warehouse.bfs_traversal,
        "DFS (поиск в глубину)": warehouse.dfs_traversal,
        "Жадный (ближайший сосед)": warehouse.nearest_neighbor_tsp,
        "Оптимальный (складской)": warehouse.warehouse_optimal_traversal
    }
    
    print("\n" + "="*70)
    print("СРАВНЕНИЕ АЛГОРИТМОВ ОБХОДА СКЛАДА")
    print("="*70)
    
    results = []
    
    for name, algorithm in algorithms.items():
        print(f"\n{'-'*70}")
        print(f"АЛГОРИТМ: {name}")
        print('-'*70)
        
        try:
            # Измеряем время выполнения
            import time
            start_time = time.time()
            path = algorithm(start_node)
            execution_time = time.time() - start_time
            
            metrics = warehouse.calculate_path_metrics(path)
            
            print(f"Время выполнения: {execution_time:.3f} сек")
            print(f"Длина пути (шагов): {metrics['length']}")
            print(f"Посещено уникальных узлов: {metrics['unique_nodes']}")
            print(f"Повторных посещений: {metrics['repeats']}")
            print(f"Покрытие доступной зоны: {metrics['coverage']:.1%}")
            print(f"Эффективность (узлы/шаг): {metrics['efficiency']:.3f}")
            
            # Определяем качество алгоритма
            if metrics['coverage'] > 0.95:
                quality = "ОТЛИЧНО"
            elif metrics['coverage'] > 0.8:
                quality = "ХОРОШО"
            elif metrics['coverage'] > 0.6:
                quality = "УДОВЛЕТВОРИТЕЛЬНО"
            else:
                quality = "ПЛОХО"
            
            print(f"Качество обхода: {quality}")
            
            results.append((name, path, metrics, execution_time))
            
        except Exception as e:
            print(f"Ошибка выполнения: {e}")
            results.append((name, [], {}, 0))
    
    return results

def print_warehouse_stats(grid: List[List[str]], graph: nx.Graph):
    """Вывод статистики склада"""
    rows = len(grid)
    cols = len(grid[0])
    
    total_cells = rows * cols
    shelf_cells = sum(row.count('S') for row in grid)
    passage_cells = total_cells - shelf_cells
    accessible_nodes = len(graph.nodes())
    
    print("\n" + "="*70)
    print("СТАТИСТИКА СГЕНЕРИРОВАННОГО СКЛАДА")
    print("="*70)
    print(f"Размер: {rows} x {cols} = {total_cells} клеток")
    print(f"Стеллажи: {shelf_cells} клеток ({shelf_cells/total_cells:.1%})")
    print(f"Проходы: {passage_cells} клеток ({passage_cells/total_cells:.1%})")
    print(f"Доступных узлов в графе: {accessible_nodes}")
    
    # Проверяем связность
    if accessible_nodes > 0:
        is_connected = nx.is_connected(graph)
        print(f"Связность графа: {'ДА' if is_connected else 'НЕТ'}")
        
        if not is_connected:
            # Находим компоненты связности
            components = list(nx.connected_components(graph))
            print(f"Количество компонент связности: {len(components)}")
            print(f"Размеры компонент: {[len(c) for c in components]}")
    
    print("\nУСЛОВИЯ ПЕРЕДВИЖЕНИЯ:")
    print("1. Движение ТОЛЬКО по проходам ('.')")
    print("2. Стеллажи ('S') НЕДОСТУПНЫ")
    print("3. Движение ТОЛЬКО по осям (↑ ↓ ← →)")
    print("4. Диагональное движение ЗАПРЕЩЕНО")

# Основная программа
if __name__ == "__main__":
    print("ГЕНЕРАЦИЯ И АНАЛИЗ СКЛАДА")
    print("="*70)
    
    # Настройки генерации
    ROWS = 6
    COLS = 10
    AISLE_PROB = 0.3
    SEED = 42  # Для воспроизводимости
    
    print(f"\nПараметры генерации:")
    print(f"  Ряды: {ROWS}")
    print(f"  Колонки: {COLS}")
    print(f"  Вероятность прохода: {AISLE_PROB}")
    print(f"  Seed: {SEED}")
    
    # Генерируем склад
    print("\nГенерация склада...")
    generator = WarehouseGenerator(
        rows=ROWS, 
        cols=COLS, 
        aisle_prob=AISLE_PROB, 
        seed=SEED
    )
    
    grid, walkable = generator.generate_warehouse()
    graph = generator.grid_to_graph(grid, walkable)
    
    # Выводим статистику
    print_warehouse_stats(grid, graph)
    
    # Создаем навигатор
    navigator = WarehouseNavigator(graph, grid)
    
    # Выбираем стартовую вершину (обычно вход - левый верхний проход)
    start_nodes = list(graph.nodes())
    if not start_nodes:
        print("\nОШИБКА: На складе нет доступных проходов!")
        exit()
    
    # Ищем подходящую стартовую точку (предпочтительно по периметру)
    start_node = None
    perimeter_nodes = []
    
    for node in start_nodes:
        r, c = map(int, node.split('_'))
        if r == 0 or r == ROWS-1 or c == 0 or c == COLS-1:
            perimeter_nodes.append(node)
    
    if perimeter_nodes:
        start_node = perimeter_nodes[0]  # Берем первый попавшийся периметральный узел
    else:
        start_node = start_nodes[0]  # Или любой доступный
    
    print(f"\nСтартовая точка (вход): {start_node}")
    
    # Сравниваем алгоритмы
    results = compare_algorithms(navigator, start_node)
    
    # Находим лучший алгоритм
    valid_results = [r for r in results if r[2].get('efficiency', 0) > 0]
    if valid_results:
        # Сортируем по эффективности и покрытию
        valid_results.sort(key=lambda x: (
            x[2].get('coverage', 0), 
            x[2].get('efficiency', 0)
        ), reverse=True)
        
        best_name, best_path, best_metrics, best_time = valid_results[0]
        
        print("\n" + "="*70)
        print("РЕЗУЛЬТАТЫ АНАЛИЗА")
        print("="*70)
        print(f"Лучший алгоритм: {best_name}")
        print(f"Время выполнения: {best_time:.3f} сек")
        print(f"Покрытие: {best_metrics.get('coverage', 0):.1%}")
        print(f"Эффективность: {best_metrics.get('efficiency', 0):.3f} узлов/шаг")
        print(f"Длина пути: {best_metrics.get('length', 0)} шагов")
        print(f"Повторные посещения: {best_metrics.get('repeats', 0)}")
        
        # Визуализируем лучший алгоритм
        print("\nВизуализация лучшего алгоритма...")
        visualize_warehouse(
            grid, graph, best_path, start_node,
            f"Лучший алгоритм: {best_name}"
        )
        
        # Также визуализируем структуру склада
        print("\nВизуализация структуры склада...")
        visualize_warehouse(grid, graph)
        
    else:
        print("\nНет подходящих алгоритмов для данного склада.")

# Пример использования с разными конфигурациями
def run_example_configurations():
    """Запуск различных конфигураций складов"""
    print("\n" + "="*70)
    print("ПРИМЕРЫ РАЗЛИЧНЫХ КОНФИГУРАЦИЙ СКЛАДОВ")
    print("="*70)
    
    configurations = [
        ("Маленький склад", 4, 6, 0.3, 1),
        ("Средний склад", 6, 8, 0.25, 2),
        ("Большой склад", 8, 12, 0.2, 3),
        ("Склад с многими проходами", 5, 7, 0.4, 4),
        ("Плотный склад", 7, 10, 0.15, 5),
    ]
    
    for name, rows, cols, prob, seed in configurations:
        print(f"\n\n{'-'*70}")
        print(f"КОНФИГУРАЦИЯ: {name}")
        print(f"Параметры: {rows}x{cols}, p={prob}")
        print('-'*70)
        
        generator = WarehouseGenerator(rows, cols, prob, seed)
        grid, walkable = generator.generate_warehouse()
        graph = generator.grid_to_graph(grid, walkable)
        
        print_warehouse_stats(grid, graph)

# Раскомментируйте для запуска примеров
run_example_configurations()