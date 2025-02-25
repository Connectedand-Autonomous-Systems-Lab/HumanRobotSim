import torch
import torch.nn as nn
import torch.nn.functional as F

class ImageArrayModel(nn.Module):
    def __init__(self, img_channels, img_height, img_width, array_dim, hidden_dim, output_dim):
        super(ImageArrayModel, self).__init__()
        
        # CNN for image processing
        self.cnn = nn.Sequential(
            nn.Conv2d(img_channels, 16, kernel_size=3, stride=1, padding=1),  # Output: (16, img_height, img_width)
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # Output: (16, img_height/2, img_width/2)
            
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),  # Output: (32, img_height/2, img_width/2)
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)  # Output: (32, img_height/4, img_width/4)
        )
        
        # Calculate flattened dimension of CNN output
        cnn_output_size = (img_height // 4) * (img_width // 4) * 32
        
        # Fully connected layers for combined input
        self.fc = nn.Sequential(
            nn.Linear(cnn_output_size + array_dim, hidden_dim),  # Combine image features and array input
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)  # Output layer
        )
    
    def forward(self, image, array):
        # Process the image through the CNN
        image_features = self.cnn(image)  # Shape: (batch_size, 32, img_height/4, img_width/4)
        image_features = image_features.view(image_features.size(0), -1)  # Flatten CNN output
        
        # Concatenate image features with array input
        combined_input = torch.cat((image_features, array), dim=1)  # Concatenate along feature dimension
        
        # Process combined input through fully connected layers
        output = self.fc(combined_input)
        return output
