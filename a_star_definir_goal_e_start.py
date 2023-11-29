import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import heapq

class Node:
    def __init__(self, state, parent=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def astar(start, goal, neighbors, heuristic):
    start_node = Node(state=start, cost=0, heuristic=heuristic(start))
    frontier = [start_node]
    explored = set()

    while frontier:
        current_node = heapq.heappop(frontier)

        if current_node.state == goal:
            path = []
            while current_node:
                path.append(current_node.state)
                current_node = current_node.parent
            return path[::-1]

        explored.add(current_node.state)

        for neighbor in neighbors(current_node.state):
            if neighbor in explored:
                continue

            cost = current_node.cost + 1  # assume que todos os movimentos têm o mesmo custo
            heuristic_value = heuristic(neighbor)
            new_node = Node(state=neighbor, parent=current_node, cost=cost, heuristic=heuristic_value)

            if new_node not in frontier:
                heapq.heappush(frontier, new_node)

    return None  # Se não encontrar um caminho

def animate_path(path, start, goal, walls, maze_shape):
    cmap = ListedColormap(['white', 'black', 'green', 'red', 'blue', 'gray'])

    fig, ax = plt.subplots()
    ax.set_xlim(0, maze_shape[1])
    ax.set_ylim(0, maze_shape[0])
    ax.set_aspect('equal')

    maze = [[0] * maze_shape[1] for _ in range(maze_shape[0])]
    for i in range(maze_shape[0]):
        for j in range(maze_shape[1]):
            maze[i][j] = Rectangle((j, maze_shape[0] - i - 1), 1, 1, color='white', ec='black')
            ax.add_patch(maze[i][j])

    start_rect = Rectangle((start[1], maze_shape[0] - start[0] - 1), 1, 1, color='green', ec='black')
    goal_rect = Rectangle((goal[1], maze_shape[0] - goal[0] - 1), 1, 1, color='red', ec='black')
    ax.add_patch(start_rect)
    ax.add_patch(goal_rect)

    for wall in random_walls:
        maze[wall[0]][wall[1]].set_facecolor('black')

    counter_text = ax.text(0.5, 1.01, '', transform=ax.transAxes, ha='center')

    def update(frame):
        current_state = path[frame]
        maze[current_state[0]][current_state[1]].set_facecolor('blue')
        counter_text.set_text(f'Movimentos: {frame }')

    ani = animation.FuncAnimation(fig, update, frames=len(path), repeat=False)
    plt.show()

# Exemplo de uso:

def neighbors(state, walls):
    x, y = state
    possible_moves = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]  # movimentos para a direita, esquerda, cima, baixo
    valid_moves = [(x, y) for x, y in possible_moves if 0 <= x < maze_shape[0] and 0 <= y < maze_shape[1] and (x, y) not in walls]
    return valid_moves

def heuristic(state, goal):
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

def generate_random_walls(maze_shape, num_walls):
    walls = set()

    # Adiciona paredes aleatórias até atingir o número desejado
    while len(walls) < num_walls:
        wall_candidate = (random.randint(0, maze_shape[0] - 1), random.randint(0, maze_shape[1] - 1))

        # Garante que as paredes não estejam na posição de início ou destino
        if wall_candidate != start and wall_candidate != goal:
            walls.add(wall_candidate)

    return list(walls)

# Aumente o tamanho do tabuleiro
maze_shape = (20, 20)

# Posições inicial e final
start = (2, 2)
goal = (18, 18)


#Quantidade de paredes
num_walls = 100
#Geração aleatória da localização das paredes
random_walls = generate_random_walls(maze_shape, num_walls)

path = astar(start, goal, lambda state: neighbors(state, random_walls), lambda state: heuristic(state, goal))
animate_path(path, start, goal, random_walls, maze_shape)