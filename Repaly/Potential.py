import numpy as np
import matplotlib.pyplot as plt


# Attractive potential function
def attractive_potential(x, y, goal, alpha):
    return 0.5 * alpha * (np.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2)) ** 2


# Repulsive potential function
def repulsive_potential(x, y, obstacles, beta, r0):
    potential = 0
    for obstacle in obstacles:
        distance = np.sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2)
        if distance < r0:
            potential += 0.5 * beta * (1 / (distance - r0)) ** 2
    return potential


# Compute gradients of the potential fields
def compute_gradients(x, y, goal, obstacles, alpha, beta, r0):
    # Gradient of the attractive potential field
    grad_att_x = alpha * (x - goal[0])
    grad_att_y = alpha * (y - goal[1])

    # Gradient of the repulsive potential field
    grad_rep_x = 0
    grad_rep_y = 0
    for obstacle in obstacles:
        distance = np.sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2)
        if distance < r0:
            grad_rep_x += -beta * (x - obstacle[0]) / (distance ** 3)
            grad_rep_y += -beta * (y - obstacle[1]) / (distance ** 3)
        elif distance < r0 + 1:
            grad_rep_x += beta * (x - obstacle[0]) / (distance ** 3)
            grad_rep_y += beta * (y - obstacle[1]) / (distance ** 3)

    # Combine gradients from both potential fields
    grad_x = grad_att_x + grad_rep_x
    grad_y = grad_att_y + grad_rep_y
    return grad_x, grad_y


# Path planning function
def path_planning(start, goal, obstacles, alpha, beta, r0, max_iter=1000):
    x, y = start
    path = [(x, y)]

    for _ in range(max_iter):
        grad_x, grad_y = compute_gradients(x, y, goal, obstacles, alpha, beta, r0)

        # Calculate next position
        x_new = x - 0.1 * grad_x
        y_new = y - 0.1 * grad_y

        # Update the path
        path.append((x_new, y_new))

        # Check if the goal is reached
        if np.sqrt((x_new - goal[0]) ** 2 + (y_new - goal[1]) ** 2) < 0.1:
            break

        # Update current position
        x, y = x_new, y_new

    return np.array(path)


if __name__ == "__main__":
    # Set target point and obstacles
    goal = np.array([8, 8])  # Target point coordinates
    obstacles = np.array([[3, 3], [6, 6], [7, 2]])  # Obstacle coordinates

    # Potential field parameters
    alpha = 1  # Weight of the attractive potential field
    beta = 100  # Weight of the repulsive potential field
    r0 = 1  # Influence range of repulsive potential field

    # Create grid
    x_range = np.linspace(0, 10, 100)
    y_range = np.linspace(0, 10, 100)
    X, Y = np.meshgrid(x_range, y_range)

    # Run the path planning
    start = np.array([0, 0])  # Start point coordinates
    path = path_planning(start, goal, obstacles, alpha, beta, r0)

    # Visualize the result
    plt.figure(figsize=(8, 8))
    plt.plot(path[:, 0], path[:, 1], label="Path", c="blue")
    plt.scatter(goal[0], goal[1], color='green', label="Goal")
    plt.scatter(obstacles[:, 0], obstacles[:, 1], color='red', label="Obstacles")
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Path Planning using Potential Field Method')
    plt.grid(True)
    plt.show()
