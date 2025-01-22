import torch.nn as nn
import MinkowskiEngine as ME
import numpy as np
import torch


class ExampleNetwork(ME.MinkowskiNetwork):

    def __init__(self, in_feat, out_feat, D):
        super(ExampleNetwork, self).__init__(D)
        self.conv1 = nn.Sequential(
            ME.MinkowskiConvolution(in_channels=in_feat, out_channels=64, kernel_size=3, stride=2, dilation=1, bias=False, dimension=D),
            ME.MinkowskiBatchNorm(64),
            ME.MinkowskiReLU(),
        )
        self.conv2 = nn.Sequential(
            ME.MinkowskiConvolution(in_channels=64, out_channels=128, kernel_size=3, stride=2, dimension=D),
            ME.MinkowskiBatchNorm(128),
            ME.MinkowskiReLU(),
        )
        self.pooling = ME.MinkowskiGlobalPooling()
        self.linear = ME.MinkowskiLinear(128, out_feat)

    def forward(self, x):
        out = self.conv1(x)
        out = self.conv2(out)
        out = self.pooling(out)
        return self.linear(out)


# Arbitrary data loader function
def data_loader(batch_size=4, num_points=100, num_features=3, num_classes=5):
    coords = []
    feat = []
    labels = []
    for i in range(batch_size):
        batch_coords = np.random.randint(0, 100, (num_points, 2))  # Random 2D coordinates
        batch_coords = np.hstack((np.full((num_points, 1), i), batch_coords))  # Add batch index
        coords.append(batch_coords)
        feat.append(np.random.rand(num_points, num_features))  # Random features
        labels.append(np.random.randint(0, num_classes))  # Random label

    coords = np.vstack(coords)
    feat = np.vstack(feat)
    labels = np.array(labels)

    return coords, feat, labels


# loss and network
criterion = nn.CrossEntropyLoss()
net = ExampleNetwork(in_feat=3, out_feat=5, D=2)
print(net)

# a data loader must return a tuple of coords, features, and labels.
coords, feat, label = data_loader()
coords = torch.tensor(coords, dtype=torch.int32)
feat = torch.tensor(feat, dtype=torch.float32)
input = ME.SparseTensor(feat, coordinates=coords)

# Forward
output = net(input)

# Loss
loss = criterion(output.F, torch.tensor(label, dtype=torch.long))
print(f"Loss: {loss.item()}")
