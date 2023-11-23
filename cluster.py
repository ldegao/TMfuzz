import os
import pdb

import cv2
import numpy as np
import random
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from sklearn.decomposition import PCA
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from scipy.interpolate import splprep, splev, interp1d

# Global Variables
colors = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (0, 255, 255), (255, 0, 255), (192, 192, 192), (128, 0, 0),
    (128, 128, 0), (0, 128, 0)
]


# CNN Model Initialization
def create_cnn_model():
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=(256, 256, 4)),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(128, activation='relu'),
    ])
    return model


# PCA Initialization
def create_pca():
    return PCA(n_components=2)


# DBSCAN Initialization
def create_dbscan():
    return DBSCAN(eps=0.5, min_samples=5)


# Data Generation

def test_create_trace(num_points=25, noise_level=0.5, max_weight=10):
    start_x = random.uniform(-64, 64)
    start_y = random.uniform(-64, 64)
    print(start_x,start_y)
    direction_x = random.choice([random.uniform(-2, -1), random.uniform(1, 2)])
    direction_y = random.choice([random.uniform(-2, -1), random.uniform(1, 2)])
    trace = []
    for i in range(num_points):
        noise_x = random.uniform(-noise_level, noise_level)
        noise_y = random.uniform(-noise_level, noise_level)
        weight = random.uniform(1, max_weight)
        point_x = start_x + direction_x * i + noise_x
        point_y = start_y + direction_y * i + noise_y
        point = (point_x, point_y, weight)
        trace.append(point)
    return np.array(trace)


def test_create_trace_graph(num_traces, num_points=25):
    traces = [test_create_trace(num_points=num_points) for _ in range(num_traces)]
    return np.array(traces)


def normalize_points(points, origin_point):
    points_array = np.array(points)
    origin_array = np.array(origin_point)
    normalized_points = points_array - origin_array
    return normalized_points


def shift_float(number, mod_value=256):
    shifted_number = (number + mod_value / 2) % mod_value
    return shifted_number


# Image Drawing
def draw_picture(points, img_size=(256, 256), color=(255, 255, 255), base_image=None):
    if base_image is None:
        img = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)
    else:
        img = base_image
    points = np.atleast_2d(points)
    if len(points) > 2:
        x = [shift_float(p[0], 256) for p in points]
        y = [shift_float(p[1], 256) for p in points]
        weight = []
        x, y, weight = remove_duplicate_adjacent_points(x, y, weight)
        x, y = uniform_sampling(x, y, )
        # x = normalize_points(x, x[0])
        # y = normalize_points(y, y[0])
        for i in range(len(x) - 1):
            cv2.line(img,
                     (int(x[i]), int(y[i])),
                     (int(x[i + 1]), int(y[i + 1])),
                     color=color,
                     thickness=10)
            # print("line:", (int(new_points[0][i]), int(new_points[1][i])),
            #       (int(new_points[0][i + 1]), int(new_points[1][i + 1])))
    else:
        cv2.line(img,
                 (int(points[0, 0]), int(points[0, 1])),
                 (int(points[1, 0]), int(points[1, 1])),
                 color=color,
                 thickness=4)
    return img


def draw_curve(trace, img_size=(256, 256), color=(255, 255, 255), base_image=None):
    if base_image is None:
        img = np.zeros((img_size[0], img_size[1], 4), dtype=np.uint8)
    else:
        img = base_image
    if len(trace) > 2:
        x = [shift_float(p[0], 256) for p in trace]
        y = [shift_float(p[1], 256) for p in trace]
        weights = [p[2] for p in trace]
        x, y, weights = remove_duplicate_adjacent_points(x, y, weights)
        x, y, weights = uniform_sampling_with_weights(x, y, weights)
        # x = normalize_points(x, x[0])
        # y = normalize_points(y, y[0])
        try:
            tck, u = splprep([x, y], s=0)
        except ValueError:
            print("ValueError")
            pdb.set_trace()
        u_new = np.linspace(u.min(), u.max(), 100)
        new_points = splev(u_new, tck, der=0)
        weight_interpolator = interp1d(u, weights, kind='linear')
        new_weights = weight_interpolator(u_new)
        i = 0
        try:
            for new_point in new_points:
                img[int(new_point[0]), int(new_point[1]), :3] = color
                img[int(new_point[0]), int(new_point[1]), 3] = new_weights[i]
                i = i + 1
        except IndexError:
            print("IndexError")
            pdb.set_trace()
    return img


def interpolate_line_weights(start_point, end_point, num_points=25):
    x0, y0, w0 = start_point
    x1, y1, w1 = end_point
    x_values = np.linspace(x0, x1, num_points)
    y_values = np.linspace(y0, y1, num_points)
    weight_values = np.linspace(w0, w1, num_points)

    return x_values, y_values, weight_values


def remove_duplicate_adjacent_points(x, y, weights):
    if len(x) != len(y):
        raise ValueError("x,y not equal length")
    x = np.array(x)
    y = np.array(y)
    if len(weights) > 0:
        weights = np.array(weights)
    keep = np.ones(len(x), dtype=bool)
    for i in range(1, len(x)):
        if x[i] == x[i - 1] and y[i] == y[i - 1]:
            keep[i] = False
    if len(weights) > 0:
        return x[keep], y[keep], weights[keep]
    else:
        return x[keep], y[keep], weights


def uniform_sampling(x, y, num_points=25):
    x = np.array(x)
    y = np.array(y)
    if len(x) != len(y):
        raise ValueError("The lengths of x and y must be the same.")
    total_points = len(x)
    if total_points < num_points:
        return x, y
    # Calculate sampling interval
    indices = np.linspace(0, total_points - 1, num_points, dtype=int)

    # Sample using slicing
    return x[indices], y[indices]


def uniform_sampling_with_weights(x, y, weights, num_points=25):
    x = np.array(x)
    y = np.array(y)
    weights = np.array(weights)
    if len(x) != len(y) or len(x) != len(weights):
        raise ValueError("Lengths of x, y, and weights must be the same.")
    total_points = len(x)
    if total_points < num_points:
        return x, y, weights
    indices = np.linspace(0, total_points - 1, num_points, dtype=int)
    return x[indices], y[indices], weights[indices]


# Feature Transformation
def transform_traces_to_features(model, pca, accumulated_trace_graphs):
    trace_images = []
    for tg in accumulated_trace_graphs:
        img = np.zeros((256, 256, 4), dtype=np.uint8)
        for j, trace in enumerate(tg):
            color = colors[j % len(colors)]
            img = draw_curve(trace, img_size=(256, 256), color=color, base_image=img)
        trace_images.append(img)

    trace_images = np.array(trace_images)

    features = model.predict(trace_images)
    return features


# Clustering and Distance Calculation
def calculate_optimal_clusters(pca_result):
    n_samples = pca_result.shape[0]
    if n_samples < 2:
        return 1
    n_clusters_range = range(2, min(10, pca_result.shape[0]))
    bic_scores = []
    for n_clusters in n_clusters_range:
        kmeans = KMeans(n_clusters=n_clusters)
        kmeans.fit(pca_result)
        labels = kmeans.labels_
        bic_score = metrics.calinski_harabasz_score(pca_result, labels)
        bic_scores.append(bic_score)
    if not bic_scores:
        return 1
    else:
        return n_clusters_range[np.argmax(bic_scores)]


def calculate_distance(model, pca, accumulated_trace_graphs):
    pca_features = transform_traces_to_features(model, pca, accumulated_trace_graphs)

    optimal_n_clusters = calculate_optimal_clusters(pca_features)

    kmeans = KMeans(n_clusters=optimal_n_clusters).fit(pca_features)
    distance_list = []

    for i in range(len(pca_features)):
        distance = np.linalg.norm(pca_features[i] - kmeans.cluster_centers_[kmeans.labels_[i]])
        distance_list.append(distance)

    return distance_list


def draw_and_save_traces(accumulated_trace_graphs, save_dir):
    for i, trace_graph in enumerate(accumulated_trace_graphs):
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        for j, trace in enumerate(trace_graph):
            color = colors[j % len(colors)]
            img = draw_picture(trace, img_size=(256, 256), color=color, base_image=img)
        filename = f"{save_dir}/combined_trace_graph_{i}.png"
        # if filename not exists, then save
        if not os.path.exists(filename):
            cv2.imwrite(filename, img)


# Main Function
def main():
    model = create_cnn_model()
    pca = create_pca()
    accumulated_trace_graphs = []
    for _ in range(10):
        traces = test_create_trace_graph(random.randint(2, 2))
        accumulated_trace_graphs.append(traces)

    distance_list = calculate_distance(model, pca, accumulated_trace_graphs)
    print(distance_list)
    # Save directory for trace graphs
    save_dir = "trace"  # Replace with your actual save path
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    draw_and_save_traces(accumulated_trace_graphs, save_dir)


if __name__ == '__main__':
    main()
