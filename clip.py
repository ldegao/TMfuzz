# clip_image_similarity.py
from transformers import CLIPProcessor, CLIPModel
from PIL import Image
import torch


def load_clip_model():
    """
    Load the CLIP model and processor.
    """
    model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
    processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
    return model, processor


def preprocess_images(image_paths, processor):
    """
    Preprocess a list of images using the CLIP processor.
    """
    images = [Image.open(path) for path in image_paths]
    return processor(images=images, return_tensors="pt", padding=True)


def extract_features(processed_images, model):
    """
    Extract features from the preprocessed images.
    """
    return model.get_image_features(**processed_images)


def calculate_similarity(features1, features2):
    """
    Calculate the cosine similarity between two feature vectors.
    """
    features1_norm = features1 / features1.norm(dim=-1, keepdim=True)
    features2_norm = features2 / features2.norm(dim=-1, keepdim=True)
    similarity = torch.matmul(features1_norm, features2_norm.T)
    return similarity


def main():
    # Load the model
    model, processor = load_clip_model()

    # Image paths
    image_path1 = "./trace/combined_trace_graph_0.png"
    image_path2 = "./trace/combined_trace_graph_1.png"

    # Preprocess images
    processed_images = preprocess_images([image_path1, image_path2], processor)

    # Extract features
    features = extract_features(processed_images, model)

    # Calculate similarity
    similarity = calculate_similarity(features[0].unsqueeze(0), features[1].unsqueeze(0))

    # Print similarity result
    print(f"Similarity between the images: {similarity.item()}")


if __name__ == "__main__":
    main()
