import math
import os
import random
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from scipy.interpolate import splprep, splev, interp1d
from torchvision import models, transforms, datasets
from sklearn.cluster import KMeans
from sklearn.metrics import pairwise_distances, silhouette_score

# Global Variables
colors = [
    (70, 70, 130), (130, 70, 70),
]


# ResNet-50 Feature Extractor Initialization
class FeatureExtractor(nn.Module):
    def __init__(self):
        super(FeatureExtractor, self).__init__()
        resnet50 = models.resnet50(pretrained=True)
        self.features = nn.Sequential(*list(resnet50.children())[:-1])  # ?????????

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)  # ???2D??
        return x


def compute_wcss(X, labels):
    k = len(np.unique(labels))
    centroids = np.array([X[labels == i].mean(axis=0) for i in range(k)])
    wcss = np.sum([np.sum((X[labels == i] - centroids[i]) ** 2) for i in range(k)])
    return wcss


def compute_inter_cluster_dist(centroids):
    dist_matrix = pairwise_distances(centroids)
    k = len(centroids)
    inter_dist = np.sum(dist_matrix) / (k * (k - 1))
    return inter_dist


def compute_silhouette_loss(X, labels):
    return -silhouette_score(X, labels)


def combined_loss(X, labels, alpha=1.0, beta=1.0):
    k = len(np.unique(labels))
    centroids = np.array([X[labels == i].mean(axis=0) for i in range(k)])

    wcss = compute_wcss(X, labels)
    inter_dist = compute_inter_cluster_dist(centroids)
    silhouette_loss = compute_silhouette_loss(X, labels)

    loss = wcss - alpha * inter_dist + beta * silhouette_loss
    return loss


# Data Generation
def test_create_trace(num_points=25, noise_level=0.5, max_weight=10):
    start_x = random.uniform(-64, 64)
    start_y = random.uniform(-64, 64)
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


def shift_scale_points_group(points_group, img_size):
    min_x, min_y = np.min(points_group[0], axis=0)
    max_x, max_y = np.max(points_group[0], axis=0)

    for points in points_group:
        min_x = min(min_x, np.min(points[:, 0]))
        min_y = min(min_y, np.min(points[:, 1]))
        max_x = max(max_x, np.max(points[:, 0]))
        max_y = max(max_y, np.max(points[:, 1]))

    if (max_x - min_x) == 0:
        scale_x = 1
    else:
        scale_x = img_size[0] / (max_x - min_x)

    if (max_y - min_y) == 0:
        scale_y = 1
    else:
        scale_y = img_size[1] / (max_y - min_y)

    scale = min(scale_x, scale_y) * 0.8

    # if (max_x - min_x) == 0:
    #     scale_x = 1
    # else:
    #     scale_x = img_size[0] / (max_x - min_x)
    #
    # if (max_y - min_y) == 0:
    #     scale_y = 1
    # else:
    #     scale_y = img_size[1] / (max_y - min_y)

    shift_x = (img_size[0] - (max_x - min_x) * scale) / 2
    shift_y = (img_size[1] - (max_y - min_y) * scale) / 2

    scaled_and_shifted_groups = []
    for points in points_group:
        points_scaled = (points - np.array([min_x, min_y])) * scale
        points_shifted = points_scaled + np.array([shift_x, shift_y])
        scaled_and_shifted_groups.append(points_shifted.astype(np.int32))

    return scaled_and_shifted_groups


def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def draw_picture(trace, img_size=(1024, 1024), color=(255, 255, 255), base_image=None):
    arrow_length = 10
    if base_image is None:
        img = np.full((img_size[0], img_size[1], 3), 255, dtype=np.uint8)
    else:
        img = base_image
    curve_points = [np.array(point)[:2] for point in trace]
    last_point = curve_points[-1]
    arrow_start_point = curve_points[0]
    for point in curve_points:
        if distance(point, last_point) >= arrow_length:
            arrow_start_point = point
    if np.array_equal(last_point, arrow_start_point):
        return img
    direction_vector = np.subtract(last_point, arrow_start_point)
    direction_vector = direction_vector / np.linalg.norm(direction_vector)
    short_start_point = np.add(last_point, -direction_vector * arrow_length).astype(int)

    if len(curve_points) >= 2:
        cv2.polylines(img, [np.array(curve_points)], isClosed=False, color=color, thickness=2)
        cv2.arrowedLine(img,
                        tuple(short_start_point),
                        tuple(last_point),
                        color=color,
                        thickness=2,
                        tipLength=1)
    return img


def draw_curve(trace, img_size=(256, 256), color=(255, 255, 255), base_image=None):
    if base_image is None:
        img = np.zeros((img_size[0], img_size[1], 4), dtype=np.uint8)
    else:
        img = base_image
    if len(trace) > 4:
        x = [shift_float(p[0], 256) for p in trace]
        y = [shift_float(p[1], 255) for p in trace]

        weights = [p[2] for p in trace]
        x, y, weights = remove_duplicate_adjacent_points(x, y, weights)
        x, y, weights = uniform_sampling_with_weights(x, y, weights)
        x = normalize_points(x, x[0])
        y = normalize_points(y, y[0])
        try:
            tck, u = splprep([x, y], s=0)
        except ValueError:
            print("ValueError")
            return img
        except TypeError:
            print("TypeError")
            return img
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
        return img

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
    indices = np.linspace(0, total_points - 1, num_points, dtype=int)

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


# Feature Transformation using ResNet-50
def transform_traces_to_features(model, accumulated_trace_graphs):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    trace_images = []

    for tg in accumulated_trace_graphs:
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        for j, trace in enumerate(tg):
            color = colors[j % len(colors)]
            img = draw_curve(trace, img_size=(256, 256), color=color, base_image=img)
        trace_images.append(img)

    trace_images = np.array(trace_images)

    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    trace_images = torch.stack([transform(img) for img in trace_images]).to(device)

    with torch.no_grad():
        features = model(trace_images)

    return features.cpu().numpy()


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
        bic_score = compute_bic(kmeans, pca_result)
        bic_scores.append(bic_score)
    if not bic_scores:
        return 1
    else:
        return n_clusters_range[np.argmax(bic_scores)]


def compute_bic(kmeans, X):
    N, d = X.shape
    k = kmeans.n_clusters
    centers = kmeans.cluster_centers_
    labels = kmeans.labels_
    m = np.bincount(labels)
    cl_var = (1.0 / (N - k) / d) * sum([np.sum((X[np.where(labels == i)] - centers[i]) ** 2) for i in range(k)])
    const_term = 0.5 * k * np.log(N) * (d + 1)
    bic = np.sum([m[i] * np.log(m[i]) -
                  m[i] * np.log(N) -
                  ((m[i] * d) / 2) * np.log(2 * np.pi * cl_var) -
                  ((m[i] - 1) * d / 2) for i in range(k)]) - const_term
    return bic


def calculate_distance(model, accumulated_trace_graphs):
    pca_features = transform_traces_to_features(model, accumulated_trace_graphs)

    optimal_n_clusters = calculate_optimal_clusters(pca_features)

    kmeans = KMeans(n_clusters=optimal_n_clusters).fit(pca_features)
    distance_list = []

    for i in range(len(pca_features)):
        distance = np.linalg.norm(pca_features[i] - kmeans.cluster_centers_[kmeans.labels_[i]])
        distance_list.append(distance)

    return distance_list


def draw_and_save_traces(accumulated_trace_graphs, save_dir):
    for i, trace_graph in enumerate(accumulated_trace_graphs):
        new_trace_graph = np.array([np.array([point[:2] for point in trace]) for trace in trace_graph])
        trace_graph_points = shift_scale_points_group(np.array(new_trace_graph), (1024, 1024))
        img = np.full((1024, 1024, 3), 255, dtype=np.uint8)
        for j, trace in enumerate(trace_graph_points):
            color = colors[j % len(colors)]
            img = draw_picture(trace, color=color, base_image=img)
        filename = f"{save_dir}/combined_trace_graph_{i}.png"
        if not os.path.exists(filename):
            cv2.imwrite(filename, img)


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = FeatureExtractor().to(device)
    accumulated_trace_graphs = []
    for _ in range(10):
        traces = test_create_trace_graph(random.randint(2, 2))
        accumulated_trace_graphs.append(traces)

    distance_list = calculate_distance(model, accumulated_trace_graphs)
    print(distance_list)
    save_dir = "trace"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    draw_and_save_traces(accumulated_trace_graphs, save_dir)


if __name__ == '__main__':
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    main()
