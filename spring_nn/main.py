import torch
import torch.nn as nn
from torch_geometric import nn as geom_nn
from torch_geometric.data import DataLoader
from torch_geometric.nn import GCNConv
import torch.nn.functional as F
import random
from model import SimpleGraphNetwork

import os

from dataset import GraphDataset

from evaluation import evaluate

device = 'cuda' if torch.cuda.is_available() else 'cpu'

action_dim = 6
particle_dim = 2
input_dim = particle_dim + action_dim 
output_dim = particle_dim
edge_features = 1

dataset = GraphDataset()

FIGURES_DIR='figures'
if not os.path.exists(FIGURES_DIR):
    os.mkdir(FIGURES_DIR)


input_features = 6 # One action

model = SimpleGraphNetwork(input_dim, output_dim).to(device)

lr = 1e-3

opt = torch.optim.Adam(model.parameters(), lr=lr)

epochs = 1000
num_examples = len(dataset)
log_interval = num_examples // 10
gradient_step = 16
save_interval = num_examples // 2

for epoch in range(1, epochs + 1):
    losses = 0
    for elem_id in range(1, num_examples + 1):
        random_elem = random.randint(0, num_examples-1)
        if elem_id % gradient_step == 0:
            opt.zero_grad()

        s1, s2 = dataset[random_elem]

        s1 = s1.to(device)
        s2 = s2.to(device)

        s1.x = (s1.x / 100.0)
        s2.x = (s2.x / 100.0)

        ground_truth = s2.x

        out = model(s1)
        loss = F.mse_loss(out, ground_truth)
        
        losses += loss

        if elem_id % gradient_step == 0:
            losses /= gradient_step
            losses.backward()
            opt.step()
            losses = 0

        if elem_id % log_interval == 0:
            print(f"Epoch: {epoch} {elem_id}/{num_examples} Loss: {loss.item():.3}")

        if elem_id % save_interval == 0:
            print("Saving model...")
            torch.save(model, 'model.pt')
            #torch.save(model.state_dict(), 'model.pt')

    if epoch % 5 == 0:
        evaluate(model, epoch, FIGURES_DIR)