"""Portable inference architectures for schema-2 ``independent_hrm`` bundles.

Layer names and causal operations follow the HRM training model zoo. Parameters
come from the checkpoint, not from the selected file/folder's name. No sibling
training repository, dataset, optimizer or validation NPZ is needed at runtime.
"""
from __future__ import annotations

import math

import torch
from torch import nn
from torch.nn import functional as F


MODEL_NAMES = (
    'mlp', 'cnn', 'convmixer', 'resnet', 'lstm', 'gru', 'tcn',
    'transformer', 'kalmannet', 'small_gru', 'residual_tcn',
)

_PARAMETER_KEYS = {
    'mlp': {'hidden_dims'}, 'cnn': {'channels', 'kernel_size', 'num_layers'},
    'convmixer': {'channels', 'num_blocks'}, 'resnet': {'channels', 'num_blocks'},
    'tcn': {'channels', 'kernel_size', 'dilations'},
    'residual_tcn': {'channels', 'dilations'},
    'transformer': {'d_model', 'nhead', 'dim_feedforward', 'dropout', 'num_layers'},
    **{name: {'hidden_sizes', 'projection_dim'}
       for name in ('lstm', 'gru', 'kalmannet', 'small_gru')},
}


def _transformer_gelu(x):
    # Schema-2 archived predictions use tanh GELU, unlike the earlier joint-head
    # model zoo's exact GELU. A callable also prevents PyTorch's fused exact-GELU
    # encoder fastpath from silently substituting the wrong activation.
    return F.gelu(x, approximate='tanh')


class CausalConv1d(nn.Conv1d):
    def __init__(self, in_channels, out_channels, kernel_size, dilation=1, groups=1):
        super().__init__(in_channels, out_channels, kernel_size,
                         dilation=dilation, groups=groups, padding=0)
        self.left_padding = dilation * (kernel_size - 1)

    def forward(self, x):
        return super().forward(F.pad(x, (self.left_padding, 0)))


class ChannelLayerNorm(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.norm = nn.LayerNorm(channels)

    def forward(self, x):
        return self.norm(x.transpose(1, 2)).transpose(1, 2)


class ResidualBlock(nn.Module):
    def __init__(self, channels, dilation=1):
        super().__init__()
        self.path = nn.Sequential(
            CausalConv1d(channels, channels, 3, dilation=dilation),
            nn.ReLU(), CausalConv1d(channels, channels, 3, dilation=dilation),
        )

    def forward(self, x):
        return F.relu(x + self.path(x))


class ConvMixerBlock(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.depthwise = nn.Sequential(
            CausalConv1d(channels, channels, 5, groups=channels),
            nn.GELU(), ChannelLayerNorm(channels),
        )
        self.pointwise = nn.Sequential(
            nn.Conv1d(channels, channels, 1), nn.GELU(), ChannelLayerNorm(channels),
        )

    def forward(self, x):
        return self.pointwise(x + self.depthwise(x))


class ConvolutionEncoder(nn.Module):
    def __init__(self, input_dim, kind, params):
        super().__init__()
        channels = int(params.get('channels', 32 if kind == 'residual_tcn' else 64))
        kernel = int(params.get('kernel_size', 3))
        self.output_dim = channels
        if kind == 'cnn':
            layers = []
            for index in range(int(params.get('num_layers', 2))):
                layers.extend((CausalConv1d(input_dim if index == 0 else channels,
                                           channels, kernel), nn.ReLU()))
        elif kind == 'convmixer':
            layers = [CausalConv1d(input_dim, channels, 3), nn.GELU(),
                      ChannelLayerNorm(channels)]
            layers.extend(ConvMixerBlock(channels)
                          for _ in range(int(params.get('num_blocks', 1))))
        elif kind == 'resnet':
            layers = [CausalConv1d(input_dim, channels, 3), nn.ReLU()]
            layers.extend(ResidualBlock(channels)
                          for _ in range(int(params.get('num_blocks', 2))))
        elif kind == 'tcn':
            layers = []
            for index, dilation in enumerate(params.get('dilations', (1, 2, 4))):
                layers.extend((CausalConv1d(input_dim if index == 0 else channels,
                                           channels, kernel, dilation=int(dilation)),
                               nn.ReLU()))
        elif kind == 'residual_tcn':
            layers = [nn.Conv1d(input_dim, channels, 1), nn.ReLU()]
            layers.extend(ResidualBlock(channels, dilation=int(dilation))
                          for dilation in params.get('dilations', (1, 2, 4)))
        else:
            raise ValueError(f'Unsupported convolution architecture: {kind}')
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x.transpose(1, 2)).transpose(1, 2)


class RecurrentEncoder(nn.Module):
    def __init__(self, input_dim, kind, params):
        super().__init__()
        defaults = {'lstm': ([60, 30], 32), 'gru': ([72, 36], None),
                    'kalmannet': ([96], 96), 'small_gru': ([64], None)}
        hidden, projection = defaults[kind]
        hidden = [int(value) for value in params.get('hidden_sizes', hidden)]
        projection = params.get('projection_dim', projection)
        if len(hidden) not in (1, 2) or min(hidden) < 1:
            raise ValueError('Only the saved one/two-stage recurrent encoders are supported.')
        cell = nn.LSTM if kind == 'lstm' else nn.GRU
        self.first = cell(input_dim, hidden[0], batch_first=True)
        self.second = cell(hidden[0], hidden[1], batch_first=True) if len(hidden) == 2 else None
        self.projection = (nn.Identity() if projection is None else
                           nn.Sequential(nn.Linear(hidden[-1], int(projection)), nn.ReLU()))
        self.output_dim = hidden[-1] if projection is None else int(projection)

    def forward(self, x):
        # State is reset for each sliding window, as during training.
        x, _ = self.first(x)
        if self.second is not None:
            x, _ = self.second(x)
        return self.projection(x)


class TransformerEncoder(nn.Module):
    def __init__(self, input_dim, window_samples, params):
        super().__init__()
        # Schema-2 deployed bundles currently contain a single named `layer`.
        if int(params.get('num_layers', 1)) != 1:
            raise ValueError('This bundle adapter supports the single-layer HRM Transformer.')
        dimension = int(params.get('d_model', 32))
        if dimension % 2:
            raise ValueError('Sinusoidal position encoding requires even d_model.')
        self.output_dim = dimension
        self.projection = nn.Linear(input_dim, dimension)
        self.layer = nn.TransformerEncoderLayer(
            d_model=dimension, nhead=int(params.get('nhead', 2)),
            dim_feedforward=int(params.get('dim_feedforward', 64)),
            dropout=float(params.get('dropout', 0.1)), activation=_transformer_gelu,
            batch_first=True,
        )
        position = torch.arange(window_samples, dtype=torch.float32).unsqueeze(1)
        divisor = torch.exp(torch.arange(0, dimension, 2).float()
                            * (-math.log(10000.0) / dimension))
        encoding = torch.zeros(window_samples, dimension)
        encoding[:, 0::2] = torch.sin(position * divisor)
        encoding[:, 1::2] = torch.cos(position * divisor)
        self.register_buffer('position_encoding', encoding.unsqueeze(0))
        self.register_buffer('causal_mask', torch.triu(
            torch.ones(window_samples, window_samples, dtype=torch.bool), diagonal=1))

    def forward(self, x):
        length = x.shape[1]
        x = self.projection(x) + self.position_encoding[:, :length].to(dtype=x.dtype)
        return self.layer(x, src_mask=self.causal_mask[:length, :length])


class BranchNetwork(nn.Module):
    def __init__(self, name, input_dim, output_dim, window_samples, params):
        super().__init__()
        self.name = name
        unknown = set(params) - _PARAMETER_KEYS.get(name, set())
        if unknown:
            raise ValueError(f'Unsupported {name} architecture parameters: {sorted(unknown)}')
        if name == 'mlp':
            dimensions = [input_dim, *params.get('hidden_dims', (208, 104, 52))]
            layers = []
            for left, right in zip(dimensions[:-1], dimensions[1:]):
                layers.extend((nn.Linear(int(left), int(right)), nn.ReLU()))
            self.encoder = nn.Sequential(*layers)
            hidden = dimensions[-1]
        elif name in ('cnn', 'convmixer', 'resnet', 'tcn', 'residual_tcn'):
            self.encoder = ConvolutionEncoder(input_dim, name, params)
            hidden = self.encoder.output_dim
        elif name in ('lstm', 'gru', 'kalmannet', 'small_gru'):
            self.encoder = RecurrentEncoder(input_dim, name, params)
            hidden = self.encoder.output_dim
        elif name == 'transformer':
            self.encoder = TransformerEncoder(input_dim, window_samples, params)
            hidden = self.encoder.output_dim
        else:
            raise ValueError(f'Unknown HRM architecture {name!r}; supported: {MODEL_NAMES}')
        self.head = nn.Linear(int(hidden), output_dim)

    def forward(self, x):
        encoded = self.encoder(x[:, -1:, :] if self.name == 'mlp' else x)
        return self.head(encoded[:, -1, :])


class IndependentHRM(nn.Module):
    def __init__(self, bundle):
        super().__init__()
        size = int(bundle['input_dim'])
        window = int(bundle['config']['window_samples'])
        self.force_net = BranchNetwork(bundle['force_name'], size, 3, window,
                                       bundle['force_params'])
        self.location_net = BranchNetwork(bundle['location_name'], size,
                                          int(bundle['num_classes']), window,
                                          bundle['location_params'])
