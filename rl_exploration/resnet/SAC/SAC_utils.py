import numpy as np
import torch
from torch import nn
from torch import distributions as pyd
import torch.nn.functional as F
import os
from collections import deque
import random
import math
from torchvision import models, transforms
from torch.utils.data import DataLoader, Dataset
from torchvision.datasets import ImageFolder
from torch.optim import Adam
import os
from PIL import Image
from torchvision.models import ResNet50_Weights


def soft_update_params(net, target_net, tau):
    for param, target_param in zip(net.parameters(), target_net.parameters()):
        target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)


def set_seed_everywhere(seed):
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)


def make_dir(*path_parts):
    dir_path = os.path.join(*path_parts)
    try:
        os.mkdir(dir_path)
    except OSError:
        pass
    return dir_path


def weight_init(m):
    """Custom weight init for Conv2D and Linear layers."""
    if isinstance(m, nn.Linear):
        nn.init.orthogonal_(m.weight.data)
        if hasattr(m.bias, "data"):
            m.bias.data.fill_(0.0)


class MLP(nn.Module):
    def __init__(
        self, input_dim, hidden_dim, output_dim, hidden_depth, output_mod=None
    ):
        super().__init__()
        self.trunk = mlp(input_dim, hidden_dim, output_dim, hidden_depth, output_mod)
        self.apply(weight_init)

    def forward(self, x):
        return self.trunk(x)


def mlp(input_dim, hidden_dim, output_dim, hidden_depth, output_mod=None):
    if hidden_depth == 0: # Creates a single linear layer directly connecting the input to the output.
        mods = [nn.Linear(input_dim, output_dim)]
    else:
        input_dim = input_dim
        mods = [nn.Linear(input_dim, hidden_dim), nn.ReLU(inplace=True)] # A linear layer from the input to the first hidden layer.
        for i in range(hidden_depth - 1):
            mods += [nn.Linear(hidden_dim, hidden_dim), nn.ReLU(inplace=True)] # Linear layers connecting hidden layers to each other.
        mods.append(nn.Linear(hidden_dim, output_dim)) # Appends a final linear layer connecting the last hidden layer to the output.
    if output_mod is not None:
        mods.append(output_mod)
    trunk = nn.Sequential(*mods)  # Uses nn.Sequential to combine the layers in mods into a single model (trunk).
    return trunk

def cnn():
    # CNN for image processing
    cnn = nn.Sequential(
        # Convolutional layer 1
        nn.Conv2d(1, 16, kernel_size=3, stride=2, padding=1),  # Output: (16, img_height/2, img_width/2)
        nn.ReLU(),
        nn.MaxPool2d(kernel_size=2, stride=2),  # Output: (16, img_height/4, img_width/4)
        
        # Convolutional layer 2
        nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),  # Output: (32, img_height/8, img_width/8)
        nn.ReLU(),
        nn.MaxPool2d(kernel_size=2, stride=2),  # Output: (32, img_height/16, img_width/16)

        # Convolutional layer 3
        nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),  # Output: (64, img_height/32, img_width/32)
        nn.ReLU(),
        nn.MaxPool2d(kernel_size=2, stride=2)  # Output: (64, img_height/64, img_width/64)
    )

    return cnn

def resnet():
    # Load pretrained ResNet-50 model
    resnet50 = models.resnet50(weights=ResNet50_Weights.DEFAULT)
    # Remove the final fully connected layer to get features before classification
    modules = list(resnet50.children())[:-1]
    resnet50 = nn.Sequential(*modules)

    # Freeze all layers initially
    for param in resnet50.parameters():
        param.requires_grad = False

    # Unfreeze some layers (e.g., the last block of ResNet)
    for param in resnet50[-1].parameters():
        param.requires_grad = True

    return resnet50


def to_np(t):
    if t is None:
        return None
    elif t.nelement() == 0:
        return np.array([])
    else:
        return t.cpu().detach().numpy()
