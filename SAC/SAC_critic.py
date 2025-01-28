import torch
from torch import nn

import SAC.SAC_utils as utils


class DoubleQCritic(nn.Module):
    """Critic network, employes double Q-learning."""

    def __init__(self, obs_dim, action_dim, hidden_dim, hidden_depth):
        super().__init__()

        combined_input_dim = obs_dim + 8*8*64
        self.Q1 = utils.mlp(combined_input_dim + action_dim, hidden_dim, 1, hidden_depth)
        self.Q2 = utils.mlp(combined_input_dim + action_dim, hidden_dim, 1, hidden_depth)
        self.cnn = utils.cnn()

        self.outputs = dict()
        self.apply(utils.weight_init)

    def forward(self, image, obs, action):

        # Check the shape -- this happens in training
        if len(image.shape) == 3 and image.shape[0] == 40:  # Detect shape [40, 512, 512]
            # Convert to [40, 1, 512, 512]
            image = image.unsqueeze(1)

        image_features = self.cnn(image)  # Shape: (batch_size, 32, img_height/4, img_width/4)
        image_features = image_features.view(image_features.size(0), -1)  # Flatten CNN output
        # Concatenate image features with array input
        obs = torch.cat((image_features, obs), dim=1)  # Concatenate along feature dimension

        assert obs.size(0) == action.size(0)

        obs_action = torch.cat([obs, action], dim=-1)

        print(f"Image shape: {image.shape}")
        print(f"Image features shape: {image_features.shape}")
        print(f"Obs shape: {obs.shape}")
        print(f"Action shape: {action.shape}")
        print(f"Obs_Action shape: {obs_action.shape}")
        
        q1 = self.Q1(obs_action)
        q2 = self.Q2(obs_action)

        self.outputs["q1"] = q1
        self.outputs["q2"] = q2

        return q1, q2

    def log(self, writer, step):
        for k, v in self.outputs.items():
            writer.add_histogram(f"train_critic/{k}_hist", v, step)
