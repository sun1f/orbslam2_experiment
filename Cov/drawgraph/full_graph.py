import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

G = nx.complete_graph(5)
nx.draw(G)
plt.savefig("8nodes.png")
plt.show()
