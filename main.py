import random
import time
import sys
import heapq


# Constants
GOAL_STATE = [1, 2, 3,
              4, 5, 6,
              7, 8, 0]  # 0 represents the empty tile
TIME_LIMIT = 30 * 60
MEMORY_LIMIT = 1 * 1024 * 1024 * 1024  # 1 GB in bytes

MOVES = {'Up': -3, 'Down': 3, 'Left': -1, 'Right': 1}

GOAL_POSITIONS = {num: (idx % 3, idx // 3) for idx, num in enumerate(GOAL_STATE)}

class PuzzleState:
    def __init__(self, board, path=[], depth=0):
        self.board = board
        self.path = path
        self.depth = depth
        self.zero_idx = board.index(0)
        self.hash = tuple(self.board)

    def is_goal(self):
        return self.board == GOAL_STATE

    def get_moves(self):
        moves = []
        row, col = divmod(self.zero_idx, 3)
        if row > 0:
            moves.append('Up')
        if row < 2:
            moves.append('Down')
        if col > 0:
            moves.append('Left')
        if col < 2:
            moves.append('Right')
        return moves

    def move(self, direction):
        new_board = self.board.copy()
        delta = MOVES[direction]
        neighbor_idx = self.zero_idx + delta
        if direction == 'Left' and self.zero_idx % 3 == 0:
            return None
        if direction == 'Right' and (self.zero_idx + 1) % 3 == 0:
            return None
        new_board[self.zero_idx], new_board[neighbor_idx] = new_board[neighbor_idx], new_board[self.zero_idx]
        return PuzzleState(new_board, self.path + [direction], self.depth + 1)

    def manhattan_distance(self):
        distance = 0
        for idx, num in enumerate(self.board):
            if num != 0:
                current_pos = (idx % 3, idx // 3)
                goal_pos = GOAL_POSITIONS[num]
                distance += abs(current_pos[0] - goal_pos[0]) + abs(current_pos[1] - goal_pos[1])
        return distance

    def __lt__(self, other):
        return (self.depth + self.manhattan_distance()) < (other.depth + other.manhattan_distance())

    def __hash__(self):
        return hash(self.hash)

    def __eq__(self, other):
        return self.hash == other.hash

    def print_puzzle(self):
        print("Here is your puzzle:")
        for i in range(0, 9, 3):
            print("|" + "|".join(str(num) if num != 0 else ' ' for num in self.board[i:i + 3]) + "|")
        print()

    def print_solution_table(self, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends):
        try:
            from tabulate import tabulate
            has_tabulate = True
        except ImportError:
            has_tabulate = False

        print("Done! Here are the results:")
        table = [
            ["Steps", "Start " + " ".join(self.path)],
            ["Time elapsed (ms)", f"{time_elapsed * 1000:.4f}"],
            ["Iterations", iterations],
            ["Total nodes", total_nodes],
            ["Nodes in memory", nodes_in_memory],
            ["Dead ends", dead_ends]
        ]
        if has_tabulate:
            print(tabulate(table, tablefmt="grid"))
        else:
            for row in table:
                print(f"{row[0]}: {row[1]}")
        print()

def generate_random_puzzle(must_be_solvable=True):
    while True:
        board = GOAL_STATE.copy()
        random.shuffle(board)
        if must_be_solvable:
            if is_solvable(board):
                return board
        else:
            return board

def is_solvable(board):
    inv_count = 0
    tiles = [num for num in board if num != 0]
    for i in range(len(tiles)):
        for j in range(i + 1, len(tiles)):
            if tiles[i] > tiles[j]:
                inv_count += 1
    return inv_count % 2 == 0

def ldfs(start_state, depth_limit):
    iterations = 0
    total_nodes = 0
    nodes_in_memory = 0
    dead_ends = 0
    start_time = time.time()
    stack = [(start_state, 0)]
    visited = set()

    while stack:
        if time.time() - start_time > TIME_LIMIT:
            return None, time.time() - start_time, iterations, total_nodes, nodes_in_memory, dead_ends

        current_state, current_depth = stack.pop()
        iterations += 1

        if current_state.is_goal():
            nodes_in_memory = len(stack) + len(visited)
            time_elapsed = time.time() - start_time
            return current_state, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends

        if current_depth < depth_limit:
            for move in reversed(current_state.get_moves()):  # Reverse for correct order in stack
                next_state = current_state.move(move)
                if next_state is None:
                    continue
                total_nodes += 1
                if next_state in visited:
                    continue
                stack.append((next_state, current_depth + 1))
                visited.add(next_state)

    nodes_in_memory = len(visited)
    time_elapsed = time.time() - start_time
    dead_ends = 1
    return None, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends

def a_star(start_state):
    iterations = 0
    total_nodes = 0
    nodes_in_memory = 0
    dead_ends = 0
    start_time = time.time()
    open_set = []
    heapq.heappush(open_set, (start_state.depth + start_state.manhattan_distance(), start_state))
    closed_set = set()

    while open_set:
        if time.time() - start_time > TIME_LIMIT:
            return None, time.time() - start_time, iterations, total_nodes, nodes_in_memory, dead_ends
        _, current_state = heapq.heappop(open_set)
        iterations += 1
        if current_state.is_goal():
            nodes_in_memory = len(open_set) + len(closed_set)
            time_elapsed = time.time() - start_time
            return current_state, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends
        closed_set.add(current_state)
        for move in current_state.get_moves():
            next_state = current_state.move(move)
            if next_state is None:
                continue
            total_nodes += 1
            if next_state in closed_set:
                continue
            heapq.heappush(open_set, (next_state.depth + next_state.manhattan_distance(), next_state))
    nodes_in_memory = len(closed_set)
    time_elapsed = time.time() - start_time
    dead_ends = 1  # No solution found
    return None, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends

def iterative_deepening_dfs(start_state, max_depth=50):
    for depth in range(max_depth):
        result, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends = ldfs(start_state, depth)
        if result:
            return result, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends
    return None, time_elapsed, iterations, total_nodes, nodes_in_memory, dead_ends

if __name__ == "__main__":
    print("Select algorithm to solve 8-puzzle:")
    print("1. LDFS (Limited Depth-First Search)")
    print("2. A*")
    choice = input("Choice: ").strip()

    if choice == '1':
        algorithm = 'LDFS'
    elif choice == '2':
        algorithm = 'A*'
    else:
        print("Invalid choice")
        sys.exit()

    must_be_solvable = input("Must puzzle be solvable? (yes/no): ").strip().lower() == 'yes'
    do_while_not_solved = input("Do while not solved? (yes/no): ").strip().lower() == 'yes'

    while True:
        # Generate puzzle
        initial_board = generate_random_puzzle(must_be_solvable)
        start_state = PuzzleState(initial_board)
        start_state.print_puzzle()
        if not is_solvable(initial_board):
            print("This puzzle is unsolvable.")
            if not do_while_not_solved:
                break
            else:
                continue

        if algorithm == 'LDFS':
            # You can set a depth limit here or make it user-defined
            depth_limit = int(input("Enter depth limit for LDFS: ").strip())
            result, time_elapsed, iterations, total_nodes_alg, nodes_in_memory, dead_ends = ldfs(start_state, depth_limit)
        elif algorithm == 'A*':
            result, time_elapsed, iterations, total_nodes_alg, nodes_in_memory, dead_ends = a_star(start_state)
        else:
            print("Unknown algorithm")
            break

        if result:
            result.print_solution_table(time_elapsed, iterations, total_nodes_alg, nodes_in_memory, dead_ends)
            break
        else:
            print("No solution found within the resource limits")
            print(f"Time elapsed: {time_elapsed * 1000:.4f}ms")
            print(f"Iterations done: {iterations}")
            print(f"Total nodes: {total_nodes_alg}")
            print(f"Nodes in memory: {nodes_in_memory}")
            print(f"Dead ends: {dead_ends}")
            print()
            if not do_while_not_solved:
                break
