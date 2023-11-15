import os

import cv2
import numpy as np
import random
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten
from sklearn.decomposition import PCA
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

# Global Variables
colors = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (0, 255, 255), (255, 0, 255), (192, 192, 192), (128, 0, 0),
    (128, 128, 0), (0, 128, 0)
]


# CNN Model Initialization
def create_cnn_model():
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=(64, 64, 3)),
        MaxPooling2D((2, 2)),
        Flatten(),
    ])
    return model


# PCA Initialization
def create_pca():
    return PCA(n_components=2)


# DBSCAN Initialization
def create_dbscan():
    return DBSCAN(eps=0.5, min_samples=5)


# Data Generation

def test_create_trace(num_points=25, noise_level=5):
    start = random.uniform(0, 32)
    direction = random.choice([-1, 1])
    trace = []
    for i in range(num_points):
        noise = random.uniform(-noise_level, noise_level)
        point = start + direction * i + noise
        trace.append([i, point])

    return np.array(trace)


def test_create_trace_graph(num_traces, num_points=25):
    traces = [test_create_trace(num_points=num_points) for _ in range(num_traces)]
    return np.array(traces)


# Image Drawing
def draw_curve(points, img_size=(64, 64), color=(255, 255, 255), base_image=None):
    if base_image is None:
        img = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)
    else:
        img = base_image

    points = np.atleast_2d(points)

    if len(points) > 1:
        if len(points) > 2:
            tck, u = splprep([points[:, 0], points[:, 1]], s=0)
            u_new = np.linspace(u.min(), u.max(), 1000)
            new_points = splev(u_new, tck, der=0)
            for i in range(len(new_points[0]) - 1):
                cv2.line(img,
                         (int(new_points[0][i]), int(new_points[1][i])),
                         (int(new_points[0][i + 1]), int(new_points[1][i + 1])),
                         color=color,
                         thickness=2)
        else:
            cv2.line(img,
                     (int(points[0, 0]), int(points[0, 1])),
                     (int(points[1, 0]), int(points[1, 1])),
                     color=color,
                     thickness=2)
    return img


# Feature Transformation
def transform_traces_to_features(model,pca, accumulated_trace_graphs):
    trace_images = []
    for tg in accumulated_trace_graphs:
        img = np.zeros((64, 64, 3), dtype=np.uint8)
        for j, trace in enumerate(tg):
            color = colors[j % len(colors)]
            img = draw_curve(trace, img_size=(64, 64), color=color, base_image=img)
        trace_images.append(img)

    trace_images = np.array(trace_images)

    features = model.predict(trace_images)

    pca_features = pca.fit_transform(features)

    return pca_features


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
        img = np.zeros((64, 64, 3), dtype=np.uint8)
        for j, trace in enumerate(trace_graph):
            color = colors[j % len(colors)]
            img = draw_curve(trace, img_size=(64, 64), color=color, base_image=img)
        filename = f"{save_dir}/combined_trace_graph_{i}.png"
        cv2.imwrite(filename, img)


# Main Function
def main():
    model = create_cnn_model()
    pca = create_pca()
    accumulated_trace_graphs = []
    for _ in range(10):
        traces = test_create_trace_graph(random.randint(2, 5))
        accumulated_trace_graphs.append(traces)
    distance_list = calculate_distance(model, pca, accumulated_trace_graphs)
    print(len(distance_list))
    # Save directory for trace graphs
    save_dir = "trace"  # Replace with your actual save path
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    draw_and_save_traces(accumulated_trace_graphs, save_dir)


if __name__ == '__main__':
    main()
