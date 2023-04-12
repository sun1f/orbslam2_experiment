import creategraph as cg
import showgraph as sg

if __name__ == '__main__':
    path = "E:\\my Python学习\\project test\\prac1\\pic\\pic\\data1.xls"
    datamatrix = cg.matrix(path)
    my_graph = Graph_Matrix()
    created_graph = cg.create_undirected_matrix(my_graph, datamatrix)
    sg.draw_undircted_graph(created_graph)
