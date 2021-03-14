import torch
import numpy as np
from glob import glob
from tqdm import tqdm
import torch_geometric


def to_tuple(string):
    return [float(x) for x in string.split(',')]


def create_points_edges(point_array):
    n = len(point_array)
    points = []
    edges = []

    point_match = dict()

    point_count = 0

    edges = []
    for ix in range(n // 3):
        point_id1 = 3 * ix
        point_id2 = 3 * ix + 1
        p1 = tuple([float(x) for x in point_array[point_id1].split(',')])
        p2 = tuple([float(x) for x in point_array[point_id2].split(',')])
        
        if p1 not in point_match:
            point_match[p1] = point_count
            point_count += 1
            p1_prev = False
        if p2 not in point_match:
            point_match[p2] = point_count
            point_count += 1
            p2_prev = False
            
        edge1 = [point_match[p1], point_match[p2]]
        edge2 = [point_match[p2], point_match[p1]]
        edges.append(edge1)
        edges.append(edge2)
        
        line = point_array[3 * ix+2]
        
    edges_set = [tuple(edge) for edge in edges]
    edges_set = set(edges_set)
    edges = np.array(list(edges_set))

    points = np.array([x for x in point_match.keys()])

    return points, edges


def build_dataset():
    # file_names = glob('../spring_simulator/simulator/saved_states_best/*')
    file_names = glob('../simulator/saved_states/*')
    data = []
    for trial in tqdm(file_names):
        all_trajectories = glob(f'{trial}/*')
        trajectory_data = []

        all_trajectories = sorted(all_trajectories, reverse=False, key = lambda x : int(x.split('_')[-1].split('.')[0]))
        for trajectory in all_trajectories:
            with open(trajectory) as f:
                read_data = f.read()
                data_array = read_data.split('\n')

                action = np.array([to_tuple(data_array[1]), to_tuple(data_array[2]), to_tuple(data_array[3])])
                point_array = read_data.split("\n")[5:]

                points, edges = create_points_edges(point_array)

                trajectory_data.append((points, edges, action))
        data.append(trajectory_data)

    print("Saving data...")
    np.save('graph_data.npy', data, allow_pickle=True)



class GraphDataset(torch.utils.data.Dataset):
    def __init__(self):
        self.data = np.load('graph_data.npy', allow_pickle=True)
        burn_in = 0

        state_action_data = []
        #print("Building dataset...")
        for trajectory in tqdm(self.data):
            for i in range(burn_in, len(trajectory) - 3):
                point1 = trajectory[i][0]
                edges1 = trajectory[i][1]
                # action1 = trajectory[i][2]
                s1 = [point1, edges1.T]

                action2 = trajectory[i+1][2]

                point2 = trajectory[i+1][0]
                edges2 = trajectory[i+1][1]
                s2 = [point2, edges2.T]

                action3 = trajectory[i+2][2]

                point3 = trajectory[i+2][0]
                edges3 = trajectory[i+2][1]
                s3 = [point3, edges3.T]

                action4 = trajectory[i+3][2]

                point4 = trajectory[i+3][0]
                edges4 = trajectory[i+3][1]
                s4 = [point4, edges4.T]

                # Multi Action prediction
                state_action_traj = [s1, action2, s2]
                # state_action_traj = [s1, action2, action3, s3]
                #state_action_traj = [s1, action2, action3, action4, s4]
                state_action_data.append(state_action_traj)

        self.state_action_data = state_action_data
        print("Done", len(self.state_action_data))

    def __getitem__(self, index):
        s1, action2, s2 = self.state_action_data[index]

        p1, edges1 = s1
        p2, edges2 = s2
        #print("Action cat", action2)
        action2 = action2.reshape(-1)[None, :].repeat(len(p1), axis=0)

        cat1 = np.concatenate((p1, action2), axis=-1)

        state_1 = torch_geometric.data.Data(x=torch.from_numpy(cat1).float(), edge_index=torch.from_numpy(edges1))
        state_2 = torch_geometric.data.Data(x=torch.from_numpy(p2).float(), edge_index=torch.from_numpy(edges2))

        return state_1, state_2


    def __len__(self):
        return len(self.state_action_data)




if __name__ == '__main__':
    build_dataset()

    dataset = GraphDataset()
    print(dataset[2])
