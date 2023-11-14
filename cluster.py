import cv2
import numpy as np
import random
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten
from sklearn.decomposition import PCA
from sklearn import metrics
from sklearn.cluster import DBSCAN
from sklearn.cluster import KMeans
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

# CNN
model = Sequential([
    Conv2D(32, (3, 3), activation='relu', input_shape=(64, 64, 1)),
    MaxPooling2D((2, 2)),
    Flatten(),
])
pca = PCA(n_components=2)

dbscan = DBSCAN(eps=0.5, min_samples=5)

colors = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (0, 255, 255),
    (255, 0, 255),
    (192, 192, 192),
    (128, 0, 0),
    (128, 128, 0),
    (0, 128, 0)
]

def test_create_trace(start, num_points=25):
    return np.array([[i, i + start] for i in range(1, num_points + 1)])


def test_create_trace_graph(num_traces, num_points=25):
    traces = [test_create_trace(random.randint(1, 5), num_points) for _ in range(num_traces)]
    return np.array(traces)


def draw_curve(points, img_size=(64, 64), color=(255, 255, 255)):
    img = np.zeros((img_size[0], img_size[1], 3), dtype=np.uint8)
    points = np.array(points)

    if len(points) > 3:
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
        for i in range(len(points) - 1):
            cv2.line(img,
                     (int(points[i, 0]), int(points[i, 1])),
                     (int(points[i + 1, 0]), int(points[i + 1, 1])),
                     color=color,
                     thickness=2)
    return img


def transform_traces_to_features(accumulated_trace_graphs,save_dir):
    trace_images = []
    for i, tg in enumerate(accumulated_trace_graphs):
        color = colors[i % len(colors)]
        img = draw_curve(tg, color=color)
        trace_images.append(img)

        plt.imshow(img)
        plt.gca().invert_yaxis()
        filename = f"{save_dir}/trace_image_{i}.png"
        plt.savefig(filename)
        plt.close()

    trace_images = np.array(trace_images).reshape(-1, 64, 64, 1)
    return model.predict(trace_images)


def calculate_optimal_clusters(pca_result):
    n_clusters_range = range(2, min(10, pca_result.shape[0]))
    bic_scores = []
    for n_clusters in n_clusters_range:
        kmeans = KMeans(n_clusters=n_clusters)
        kmeans.fit(pca_result)
        labels = kmeans.labels_
        bic_score = metrics.calinski_harabasz_score(pca_result, labels)
        bic_scores.append(bic_score)
    return n_clusters_range[np.argmax(bic_scores)]


def add_graph(accumulated_trace_graphs, new_trace_graph):
    if len(accumulated_trace_graphs) < 2:
        return 0
    features = transform_traces_to_features(accumulated_trace_graphs + [new_trace_graph])
    pca_result = pca.fit_transform(features)
    optimal_n_clusters = calculate_optimal_clusters(pca_result)
    kmeans = KMeans(n_clusters=optimal_n_clusters).fit(pca_result)
    distance = np.linalg.norm(pca_result[-1] - kmeans.cluster_centers_[kmeans.labels_[-1]])
    return distance


def calculate_distance(accumulated_trace_graphs):
    features = transform_traces_to_features(accumulated_trace_graphs)
    pca_result = pca.fit_transform(features)
    optimal_n_clusters = calculate_optimal_clusters(pca_result)
    kmeans = KMeans(n_clusters=optimal_n_clusters).fit(pca_result)
    distance_list = []
    for i in range(len(pca_result)):
        distance = np.linalg.norm(pca_result[i] - kmeans.cluster_centers_[kmeans.labels_[i]])
        distance_list.append(distance)
    return distance_list


if __name__ == '__main__':
    # curve_image = draw_curve([[10.5, 20.3], [15.6, 35.7], [40.2, 50.4],[60.2, 60.4]])
    # plt.imshow(curve_image)
    # plt.gca().invert_yaxis()
    # plt.show()
    traces = test_create_trace_graph(10)
    accumulated_trace_graphs = []
    for trace in traces:
        distance = add_graph(accumulated_trace_graphs, trace)
        print(distance)
        accumulated_trace_graphs.append(trace)
    distance_list = calculate_distance(accumulated_trace_graphs)
    print(distance_list)