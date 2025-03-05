import torch

device = torch.device(
    "cuda" if torch.cuda.is_available() else "cpu"
)  # using cuda if it is available, cpu otherwise


print(f"training using {device}")