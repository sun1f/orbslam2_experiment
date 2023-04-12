import networkx as nx
import matplotlib.pyplot as plt
import numpy as np


def draw_undircted_graph(my_graph):  # 显示无向图
    G = nx.Graph()  # 建立一个空的无向图G
    for node in my_graph.vertices:
        G.add_node(str(node))
    for edge in my_graph.edges:
        G.add_edge(str(edge[0]), str(edge[1]))

    print("nodes:", G.nodes())  # 输出全部的节点： [1, 2, 3]
    print("edges:", G.edges())  # 输出全部的边：[(2, 3)]
    print("number of edges:", G.number_of_edges())  # 输出边的数量：1

    nx.draw(G, node_color='b', edge_color='r', with_labels=True)
    plt.savefig("undirected_graph.png")
    plt.show()


def draw_directed_graph(my_graph):  # 显示有向图，带权
    G = nx.DiGraph()  # 建立一个空的无向图G
    for node in my_graph.vertices:
        G.add_node(str(node))
    G.add_weighted_edges_from(my_graph.edges_array)

    print("nodes:", G.nodes())  # 输出全部的节点
    print("edges:", G.edges())  # 输出全部的边
    print("number of edges:", G.number_of_edges())  # 输出边的数量
    nx.draw(G, node_color='b', edge_color='r', with_labels=True)
    pos = nx.spring_layout(G)
    plt.savefig("directed_graph.png")
    plt.show()
