
import torch
import torch.nn as nn
from torch_geometric import nn as geom_nn
from torch_geometric.data import DataLoader
from torch_geometric.nn import GCNConv, SAGEConv, NNConv, GATConv
import torch.nn.functional as F


class SimpleGraphNetwork(nn.Module):
    def __init__(self, num_node_features, output_features):
        super().__init__()
        self.conv1 = GATConv(num_node_features, 256)
        self.conv2 = GATConv(256, output_features)

    def forward(self, data):
        x, edge_index = data.x, data.edge_index

        x = F.relu(self.conv1(x, edge_index))
        return self.conv2(x, edge_index)
