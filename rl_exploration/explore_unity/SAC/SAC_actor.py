import torch
import math
from torch import nn
import torch.nn.functional as F
from torch import distributions as pyd

import SAC.SAC_utils as utils


class TanhTransform(pyd.transforms.Transform):
    domain = pyd.constraints.real
    codomain = pyd.constraints.interval(-1.0, 1.0)
    bijective = True
    sign = +1

    def __init__(self, cache_size=1):
        super().__init__(cache_size=cache_size)

    @staticmethod
    def atanh(x):
        return 0.5 * (x.log1p() - (-x).log1p())

    def __eq__(self, other):
        return isinstance(other, TanhTransform)

    def _call(self, x):
        return x.tanh()

    def _inverse(self, y):
        # We do not clamp to the boundary here as it may degrade the performance of certain algorithms.
        # one should use `cache_size=1` instead
        return self.atanh(y)

    def log_abs_det_jacobian(self, x, y):
        # We use a formula that is more numerically stable, see details in the following link
        # https://github.com/tensorflow/probability/commit/ef6bb176e0ebd1cf6e25c6b5cecdd2428c22963f#diff-e120f70e92e6741bca649f04fcd907b7
        return 2.0 * (math.log(2.0) - x - F.softplus(-2.0 * x))


class SquashedNormal(pyd.transformed_distribution.TransformedDistribution):
    def __init__(self, loc, scale):
        self.loc = loc   # like mean
        self.scale = scale  # like std

        self.base_dist = pyd.Normal(loc, scale)
        transforms = [TanhTransform()]
        super().__init__(self.base_dist, transforms)

    @property
    def mean(self):
        mu = self.loc
        for tr in self.transforms:
            mu = tr(mu)
        return mu


class DiagGaussianActor(nn.Module):
    """torch.distributions implementation of an diagonal Gaussian policy."""

    def __init__(self, obs_dim, action_dim, hidden_dim, hidden_depth, log_std_bounds):
        super().__init__()

        self.log_std_bounds = log_std_bounds

        # print(f"combined input dim shape: {combined_input_dim}")

        self.trunk = utils.mlp(obs_dim, hidden_dim, 2 * action_dim, hidden_depth)

        self.outputs = dict()
        self.apply(utils.weight_init)

    def forward(self, obs):

        # print(obs)
        # print(obs.max(), obs.min())
        mu, log_std = self.trunk(obs).chunk(2, dim=-1)
        # print(torch.isnan(log_std).any(), torch.isinf(log_std).any())
        # print(log_std.max(), log_std.min())
        # constrain log_std inside [log_std_min, log_std_max]
        # print(mu , log_std)  # 2 each                                | mu looks like the lin vel and ang vel  | logstd are small numbers like -0.1
        log_std = torch.tanh(log_std)
        log_std_min, log_std_max = self.log_std_bounds
        log_std = log_std_min + 0.5 * (log_std_max - log_std_min) * (log_std + 1)

        # print(torch.isnan(log_std).any(), torch.isinf(log_std).any())
        # print(log_std.max(), log_std.min())
        std = log_std.exp()

        self.outputs["mu"] = mu
        self.outputs["std"] = std

        # print(torch.isnan(std))
        # print("actor is passed")
        dist = SquashedNormal(mu, std)
        # print(dist)
        return dist

    def log(self, writer, step):
        for k, v in self.outputs.items():
            writer.add_histogram(f"train_actor/{k}_hist", v, step)
