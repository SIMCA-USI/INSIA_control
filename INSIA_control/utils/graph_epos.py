import networkx as nx
import matplotlib.pyplot as plt


def get_transitions(graph, start, end):
    edge_labels = nx.get_edge_attributes(graph, 'transition')
    path = nx.shortest_path(graph)
    path_edges = [edge_labels.get(x, edge_labels.get((x[1], x[0]))) for x in
                  zip(path[start][end], path[start][end][1:])]
    return path_edges


def create_graph():
    g = nx.DiGraph()
    g.add_nodes_from(
        ['Not ready to switch on', 'Switch on disabled', 'Ready to switch on', 'Switched on', 'Operation enabled',
         'Quick stop active', 'Fault reaction active', 'Fault'])
    g.add_edges_from([('Not ready to switch on', 'Switch on disabled'), ('Switch on disabled', 'Ready to switch on'),
                      ('Ready to switch on', 'Switch on disabled'), ('Ready to switch on', 'Switched on'),
                      ('Ready to switch on', 'Operation enabled'), ('Switched on', 'Switch on disabled'),
                      ('Switched on', 'Ready to switch on'), ('Switched on', 'Operation enabled'),
                      ('Operation enabled', 'Switch on disabled'), ('Operation enabled', 'Ready to switch on'),
                      ('Operation enabled', 'Switched on'), ('Operation enabled', 'Quick stop active'),
                      ('Quick stop active', 'Operation enabled'), ('Quick stop active', 'Switch on disabled'),
                      ('Fault reaction active', 'Fault'), ('Fault', 'Switch on disabled')])
    nx.set_edge_attributes(g, {('Not ready to switch on', 'Switch on disabled'): {"transition": 1},
                               ('Switch on disabled', 'Ready to switch on'): {"transition": 2},
                               ('Ready to switch on', 'Switch on disabled'): {"transition": 7},
                               ('Ready to switch on', 'Switched on'): {"transition": 3},
                               ('Ready to switch on', 'Operation enabled'): {"transition": 4},
                               ('Switched on', 'Switch on disabled'): {"transition": 10},
                               ('Switched on', 'Ready to switch on'): {"transition": 6},
                               ('Switched on', 'Operation enabled'): {"transition": 4},
                               ('Operation enabled', 'Switch on disabled'): {"transition": 9},
                               ('Operation enabled', 'Ready to switch on'): {"transition": 8},
                               ('Operation enabled', 'Switched on'): {"transition": 5},
                               ('Operation enabled', 'Quick stop active'): {"transition": 11},
                               ('Quick stop active', 'Operation enabled'): {"transition": 16},
                               ('Quick stop active', 'Switch on disabled'): {"transition": 12},
                               ('Fault reaction active', 'Fault'): {"transition": 15},
                               ('Fault', 'Switch on disabled'): {"transition": 15},
                               })
    return g


if __name__ == '__main__':
    g = create_graph()
    print('graph created')
    # pos = nx.spring_layout(g, scale=2)
    # edge_labels = nx.get_edge_attributes(g, 'transition')
    # nx.draw_networkx_edge_labels(g, pos, edge_labels)
    # nx.draw(g, with_labels=True)
    # plt.savefig("filename.png")
    nx.write_graphml_lxml(g, 'maxon.graphml')
