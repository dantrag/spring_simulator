import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import torch

from dataset import GraphDataset

# Load the dataset

device = 'cuda' if torch.cuda.is_available() else 'cpu'
def evaluate(model, epoch, FIGURES_DIR):
    dataset = GraphDataset()

    # Load an example
    example_id = random.randint(0, len(dataset)-1)
    s1, s2 = dataset[example_id]

    s1 = s1.to(device)
    s2 = s2.to(device)

    s1.x = (s1.x / 100.0)
    s2.x = (s2.x / 100.0)

    action = s1.x[0, 2:].cpu().data.numpy()

    # Do forward prediction
    s2_pred = model(s1)

    def visualize(points, edge_index, fig_name, action=None):
        plt.figure()
        points_np = points[:, :2].cpu().data.numpy()
        point_match = {tuple(p) : i for i, p in enumerate(points_np)}
        edges_np = edge_index.cpu().data.numpy().T

        G = nx.Graph()
        G.add_edges_from(edges_np)
        val_map = {ix : point for point, ix in point_match.items()}
        for ix, value in val_map.items():
            G.add_node(ix, pos=value)
        nx.draw(G, nx.get_node_attributes(G, 'pos'), with_labels=False, node_size=0)

        if action is not None:
            # First action
            a1 = action[:2]
            # a2 = action[2:4]

            # line = a1[:, None] + np.linspace(0, 1,100) * (a2[:, None] - a1[:, None])
            # line = line.T
            # plt.scatter(line[:,0], line[:,1])

            m1 = action[4:]
            a2 = action[2:4]

            line1 = a1[:, None] + np.linspace(0, 1,100) * (m1[:, None] - a1[:, None])
            line2 = m1[:, None] + np.linspace(0, 1,100) * (a2[:, None] - m1[:, None])
            
            line1 = line1.T
            line2 = line2.T
            
            plt.scatter(line1[:,0], line1[:,1])
            plt.scatter(line2[:,0], line2[:,1])

        plt.savefig(fig_name)
        

    #Original
    visualize(s1.x, s1.edge_index, f'{FIGURES_DIR}/{epoch}_s1.png')

    #With Line
    visualize(s1.x, s1.edge_index, f'{FIGURES_DIR}/{epoch}_s1_action.png', action)

    # Next step
    visualize(s2.x, s2.edge_index, f'{FIGURES_DIR}/{epoch}_s2.png')

    # Next step prediction
    visualize(s2_pred, s1.edge_index, f'{FIGURES_DIR}/{epoch}_prediction.png')