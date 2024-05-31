# download_model.py
from huggingface_hub import hf_hub_download

# Define the local path where the model will be saved
local_model_path = "model.pt"

# Download the model
hf_hub_download(repo_id="arnabdhar/YOLOv8-Face-Detection", filename="model.pt", cache_dir=".")

print(f"Model downloaded and saved as {local_model_path}")
