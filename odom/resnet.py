import torch
import torch.nn as nn
from torchvision import models, transforms
from torch.utils.data import DataLoader, Dataset
from torchvision.datasets import ImageFolder
from torch.optim import Adam
import os
from PIL import Image

# Load pretrained ResNet-50 model
resnet50 = models.resnet50(pretrained=True)

# Remove the final fully connected layer to get features before classification
modules = list(resnet50.children())[:-1]
resnet50 = nn.Sequential(*modules)

# Freeze all layers initially
for param in resnet50.parameters():
    param.requires_grad = False

# Unfreeze some layers (e.g., the last block of ResNet)
for param in resnet50[-1].parameters():
    param.requires_grad = True

# Set up a simple dataset (replace with your own dataset)
class SimpleDataset(Dataset):
    def __init__(self, image_folder, transform=None):
        self.image_folder = image_folder
        self.transform = transform
        self.image_paths = [os.path.join(image_folder, fname) for fname in os.listdir(image_folder)]

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        img_path = self.image_paths[idx]
        img = Image.open(img_path).convert('RGB')
        if self.transform:
            img = self.transform(img)
        return img

# Data transformation
transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# Create dataset and dataloaders
train_dataset = SimpleDataset(image_folder='path_to_your_train_data', transform=transform)
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

# Define a simple training loop
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
resnet50.to(device)

# Set up optimizer only for unfrozen layers
optimizer = Adam(resnet50.parameters(), lr=1e-4)

# Training loop (you can adjust this to your specific needs)
num_epochs = 10
for epoch in range(num_epochs):
    resnet50.train()
    running_loss = 0.0
    for inputs in train_loader:
        inputs = inputs.to(device)

        optimizer.zero_grad()

        # Forward pass
        features = resnet50(inputs)

        # Example loss (adjust to your task)
        loss = features.mean()  # dummy loss
        loss.backward()

        optimizer.step()

        running_loss += loss.item()

    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {running_loss / len(train_loader)}")

# You can now extract features by passing an image through resnet50
def extract_features(image_path):
    img = Image.open(image_path).convert('RGB')
    img = transform(img).unsqueeze(0).to(device)
    with torch.no_grad():
        features = resnet50(img)
    return features

# Example usage
image_path = 'path_to_an_image.jpg'
features = extract_features(image_path)
print(features.shape)
