import numpy as np

def create_undirected_matrix(my_graph, matrix):
    nodes = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10',
             '11', '12', '13', '14', '15', '16', '17', '18', '19', '20',
             '21', '22', '23', '24', '25', '26', '27', '28', '29', '30',
             '31', '32', '33', '34', '35', '36', '37', '38', '39', '40',
             '41', '42', '43', '44', '45', '46', '47', '48', '49', '50',
             '51', '52', '53', '54', '55', '56', '57', '58', '59', '60',
             '61', '62', '63', '64', '65', '66', '67', '68', '69', '70',
             '71', '72', '73', '74', '75', '76', '77', '78', '79', '80',
             ]
    my_graph = Graph_Matrix(nodes, matrix)
    print(my_graph)
    return my_graph


def create_directed_matrix(my_graph):  # 二维数组生成有向图
    nodes = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    inf = float('inf')
    matrix = [[0, 2, 1, 3, 9, 4, inf, inf],  # a
              [inf, 0, 4, inf, 3, inf, inf, inf],  # b
              [inf, inf, 0, 8, inf, inf, inf, inf],  # c
              [inf, inf, inf, 0, 7, inf, inf, inf],  # d
              [inf, inf, inf, inf, 0, 5, inf, inf],  # e
              [inf, inf, 2, inf, inf, 0, 2, 2],  # f
              [inf, inf, inf, inf, inf, 1, 0, 6],  # g
              [inf, inf, inf, inf, inf, 9, 8, 0]]  # h

    my_graph = Graph_Matrix(nodes, matrix)
    print(my_graph)
    return my_graph


def create_directed_graph_from_edges(my_graph):  # 用边生成有向图
    nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
    edge_list = [('A', 'F', 9), ('A', 'B', 10), ('A', 'G', 15), ('B', 'F', 2),
                 ('G', 'F', 3), ('G', 'E', 12), ('G', 'C', 10), ('C', 'E', 1),
                 ('E', 'D', 7)]

    my_graph = Graph_Matrix(nodes)
    my_graph.add_edges_from_list(edge_list)
    print(my_graph)
    return my_graph


def matrix(path):  # 读取excel表格里的数据
    table = xlrd.open_workbook(path).sheets()[6]    # 获取第6个sheet表
    row = table.nrows                               # 行数
    col = table.ncols                               # 列数
    # 生成一个nrows行ncols列，且元素均为0的初始矩阵
    datamatrix = np.zeros((row, col))
    for x in range(col):
        try:
            cols = np.matrix(table.col_values(x))   # 把list转换为矩阵进行矩阵操作
            datamatrix[:, x] = cols                 # 按列把数据存进矩阵中
        except:
            print(x)

    print(datamatrix.shape)
    return datamatrix
