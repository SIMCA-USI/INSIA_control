import networkx as nx
import matplotlib.pyplot as plt


def get_transitions(graph, start, end):
    edge_labels = nx.get_edge_attributes(graph, 'transition')
    path = nx.shortest_path(graph)
    path_edges = [edge_labels.get(x, edge_labels.get((x[1], x[0]))) for x in
                  zip(path[start][end], path[start][end][1:])]
    return path_edges


if __name__ == '__main__':
    g = nx.read_graphml('../INSIA_control/utils/maxon.graphml')
    print(type(g))
    print('graph created')
    print(get_transitions(g, 'Fault', 'Switched on'))
