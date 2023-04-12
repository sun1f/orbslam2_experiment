import networkx as nx
import matplotlib.pyplot as plt

# 创建图
G = nx.Graph()

# 增加节点
G.add_nodes_from([5, 1, 3, 7, 12, 8, 0, 2, 4, 16, 10, 14, 9, 27, 25, 22, 18, 17, 29, 28, 26, 19, 20, 24, 21, 23, 15, 13, 11, 34, 31, 33, 32, 36, 35, 30, 38, 37, 41, 39, 43, 45, 46, 48, 44, 42, 50, 55, 40, 54, 53, 51, 47, 60, 56, 49, 52, 67, 62, 57, 59, 58, 63, 64, 72, 68, 61, 74, 73, 70, 71, 66, 65, 85, 81, 96, 78, 79, 69, 83, 75, 76, 93, 94, 91, 87, 111, 124, 121, 112, 110, 105, 102, 115, 108, 116, 114, 118, 117, 113, 125, 120, 122, 123, 119, 128, 126, 133, 132, 131, 139, 135, 130, 129, 127, 142, 140, 144, 137, 155, 154, 158, 146, 151, 147, 152, 148, 157, 160, 176, 170, 164, 191, 167, 162, 189, 184, 186, 178, 180, 173, 171, 187, 273, 242, 194, 199, 185, 193, 209, 200, 211, 208, 202, 203, 197, 206, 210, 214, 204, 212, 223, 222, 219, 213, 218, 217, 221, 228, 225, 216, 215, 229, 227, 224, 220, 230, 231, 226, 241, 240, 238, 239, 232, 243, 236, 237, 235, 244, 245, 246, 233, 234, 260, 258, 264, 263, 255, 259, 257, 247, 249, 250, 248, 254, 256, 253, 252, 251,
                 262, 261, 267, 266, 276, 274, 272, 270, 265, 271, 268, 277, 275, 291, 289, 287, 288, 269, 279, 285, 283, 281, 296, 295, 280, 284, 282, 278, 307, 306, 286, 292, 290, 305, 304, 303, 302, 298, 300, 309, 310, 308, 312, 313, 314, 301, 299, 297, 293, 311, 318, 334, 317, 320, 319, 316, 315, 332, 324, 322, 330, 325, 326, 328, 321, 323, 327, 329, 335, 331, 333, 342, 336, 339, 341, 340, 343, 338, 337, 347, 345, 349, 348, 352, 353, 350, 344, 346, 355, 356, 358, 351, 357, 354, 365, 366, 379, 364, 377, 374, 360, 359, 363, 362, 361, 368, 369, 367, 370, 371, 375, 376, 395, 386, 380, 384, 378, 383, 381, 373, 372, 396, 405, 399, 382, 390, 389, 387, 385, 401, 392, 393, 391, 388, 407, 406, 400, 397, 408, 398, 394, 403, 402, 404, 409, 422, 431, 419, 426, 418, 410, 411, 420, 412, 414, 413, 424, 423, 421, 425, 415, 416, 417, 430, 439, 434, 435, 436, 432, 440, 444, 443, 438, 449, 450, 433, 428, 429, 427, 442, 441, 437, 452, 447, 445, 446, 456, 454, 451, 448, 453, 455])

# 增加权重，数据格式（节点1，节点2，权重）
e = [(0, 1, 467), (0, 2, 282), (2, 3, 271), (3, 5, 241), (3, 4, 269), (5, 7, 272), (7, 8, 229), (8, 10, 202), (8, 9, 206), (10, 12, 205), (10, 11, 194), (12, 14, 214), (12, 13, 204), (13, 15, 196), (14, 16, 208), (15, 17, 181), (17, 18, 177), (18, 19, 200), (18, 20, 184), (20, 21, 205), (21, 22, 190), (21, 23, 136), (23, 24, 212), (24, 25, 200), (24, 26, 179), (26, 27, 162), (27, 28, 203), (28, 29, 212), (28, 30, 189), (29, 31, 126), (31, 33, 138), (31, 32, 184), (33, 34, 155), (34, 35, 149), (35, 36, 207), (36, 37, 136), (37, 38, 203), (37, 39, 166), (38, 40, 114), (40, 41, 157), (40, 42, 187), (41, 43, 182), (42, 44, 186), (44, 45, 193), (45, 46, 154), (45, 47, 155), (47, 48, 140), (48, 50, 162), (48, 49, 169), (49, 51, 167), (51, 52, 181), (52, 53, 136), (53, 54, 121), (54, 55, 138), (55, 56, 143), (55, 57, 177), (57, 59, 162), (57, 58, 161), (58, 60, 154), (60, 61, 155), (61, 62, 161), (61, 63, 174), (62, 64, 104), (64, 65, 125), (65, 66, 153), (66, 67, 161), (66, 68, 155), (67, 69, 153), (69, 70, 153), (70, 72, 105), (70, 71, 167), (72, 73, 160), (73, 74, 141), (74, 75, 176), (74, 76, 173), (76, 78, 124), (76, 79, 129), (79, 81, 145), (81, 83, 190), (83, 85, 128), (85, 87, 149), (87, 91, 150), (91, 93, 129), (93, 94, 158), (94, 96, 150), (96, 102, 115), (102, 105, 149), (105, 108, 158), (108, 111, 118), (108, 110, 132), (111, 112, 146), (112, 114, 161), (112, 113, 154), (113, 115, 95), (115, 116, 140), (115, 117, 172), (116, 118, 92), (118, 120, 116), (118, 119, 113), (119, 121, 159), (121, 122, 104), (122, 124, 155), (122, 123, 141), (124, 125, 113), (125, 126, 121), (125, 127, 156), (127, 128, 164), (127, 130, 167), (127, 129, 175), (130, 131, 170), (131, 132, 121), (132, 133, 154), (133, 135, 189), (135, 137, 148), (137, 139, 166), (139, 140, 170), (140, 142, 205), (142, 144, 182), (144, 146, 221), (144, 147, 164), (147, 148, 199), (148, 151, 222), (151, 154, 180), (151, 152, 171), (152, 155, 128), (155, 157, 242), (157, 158, 209), (158, 160, 192), (160, 162, 163), (162, 164, 194), (164, 167, 193), (167, 170, 236), (167, 171, 210), (170, 173, 158), (173, 176, 201), (176, 178, 229), (178, 184, 166), (178, 180, 245), (184, 186, 198), (184, 185, 195), (186, 187, 210), (187, 189, 223), (189, 191, 203), (191, 193, 162), (193, 194, 205), (193, 197, 197), (197, 199, 209), (199, 200, 224), (200, 202, 181), (202, 203, 174), (203, 204, 218), (204, 206, 217), (206, 208, 221), (208, 209, 211), (209, 210, 224), (210, 211, 231), (210, 212, 183), (211, 213, 185), (213, 214, 154), (214, 215, 191), (215, 217, 133), (215, 216, 212), (217, 218, 214), (218, 219, 217), (218, 220, 143), (220, 221, 163), (221, 223, 203), (221, 222, 191), (223, 225, 231), (223, 224, 222), (224, 226, 201), (226, 228, 198), (226, 227, 224), (227, 229, 176), (229, 230, 196), (230, 231, 220), (231, 232, 217), (232, 233, 211), (233, 235, 211), (233, 234, 256), (234, 236, 230), (235, 237, 265), (237, 238, 247), (238, 239, 174), (239, 240, 194), (240, 242, 240), (240, 241, 230), (242, 243, 238), (242, 244, 255), (243, 245, 237), (245, 246, 259), (245, 247, 200), (247, 248, 201), (248, 249, 190), (249, 250, 198), (250, 251, 258), (251, 252, 227), (252, 253, 252), (253, 255, 182), (253, 254, 265), (255, 256, 212), (256, 257, 241), (257, 258, 235), (258, 260, 261), (258, 259, 237), (260,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     262, 246), (260, 261, 248), (262, 263, 249), (263, 264, 262), (264, 265, 283), (265, 266, 278), (266, 267, 175), (267, 268, 178), (268, 269, 234), (269, 270, 247), (269, 271, 243), (271, 272, 231), (272, 273, 269), (272, 274, 163), (274, 276, 197), (274, 275, 258), (276, 277, 278), (276, 278, 144), (278, 279, 251), (279, 280, 284), (280, 281, 260), (280, 282, 235), (282, 283, 226), (282, 284, 232), (284, 285, 243), (284, 286, 237), (285, 287, 216), (286, 288, 226), (288, 289, 241), (288, 290, 222), (289, 291, 121), (291, 292, 165), (292, 293, 202), (293, 295, 201), (295, 296, 211), (295, 297, 200), (297, 298, 168), (298, 299, 148), (299, 300, 173), (300, 302, 176), (300, 301, 198), (301, 303, 212), (302, 304, 212), (303, 305, 105), (305, 306, 145), (306, 307, 172), (307, 309, 199), (307, 308, 201), (309, 310, 205), (309, 311, 171), (311, 312, 156), (312, 313, 151), (313, 314, 173), (313, 315, 166), (314, 316, 178), (315, 317, 93), (317, 318, 196), (317, 319, 208), (318, 320, 116), (320, 321, 155), (321, 322, 155), (322, 323, 182), (323, 324, 184), (323, 325, 234), (325, 326, 207), (325, 328, 216), (325, 327, 205), (328, 329, 168), (329, 330, 218), (329, 331, 163), (331, 332, 199), (332, 334, 122), (332, 333, 234), (334, 335, 227), (335, 336, 224), (336, 338, 183), (336, 337, 228), (338, 339, 209), (339, 341, 125), (339, 340, 219), (341, 342, 254), (342, 343, 250), (343, 345, 209), (343, 344, 268), (345, 347, 231), (345, 346, 211), (347, 348, 195), (348, 349, 238), (349, 350, 263), (349, 351, 154), (351, 352, 173), (352, 353, 213), (352, 354, 238), (354, 355, 246), (354, 356, 243), (355, 357, 264), (357, 358, 253), (357, 359, 225), (358, 360, 203), (360, 362, 224), (360, 361, 217), (361, 363, 240), (363, 365, 249), (363, 364, 245), (364, 366, 225), (365, 367, 223), (367, 368, 241), (368, 369, 260), (368, 370, 141), (370, 371, 179), (371, 373, 231), (371, 372, 232), (373, 374, 225), (374, 375, 246), (374, 376, 270), (375, 377, 150), (377, 378, 183), (378, 379, 257), (378, 380, 188), (380, 381, 252), (381, 382, 258), (382, 383, 151), (383, 384, 177), (384, 386, 230), (384, 385, 231), (386, 387, 168), (387, 389, 228), (387, 388, 210), (388, 390, 163), (390, 391, 172), (391, 392, 232), (391, 393, 146), (393, 395, 224), (393, 394, 255), (395, 396, 153), (396, 397, 182), (397, 398, 222), (398, 399, 166), (399, 401, 219), (399, 400, 229), (400, 402, 156), (402, 403, 148), (403, 405, 224), (403, 404, 199), (405, 407, 152), (405, 406, 225), (407, 408, 239), (407, 409, 214), (408, 410, 157), (410, 411, 159), (410, 412, 208), (411, 413, 136), (413, 414, 224), (413, 415, 209), (415, 416, 138), (416, 417, 144), (417, 418, 204), (418, 419, 188), (418, 420, 213), (419, 421, 202), (420, 422, 231), (421, 423, 181), (422, 424, 173), (424, 426, 205), (424, 425, 204), (426, 428, 230), (426, 427, 226), (427, 429, 227), (428, 430, 205), (430, 431, 214), (431, 432, 210), (431, 433, 223), (433, 434, 235), (433, 435, 242), (434, 436, 159), (436, 437, 192), (437, 439, 211), (437, 438, 222), (438, 440, 234), (440, 442, 195), (440, 441, 247), (442, 444, 180), (442, 443, 212), (444, 445, 206), (444, 446, 225), (445, 447, 210), (446, 448, 211), (447, 449, 193), (449, 450, 212), (450, 452, 132), (450, 451, 220), (452, 453, 131), (453, 454, 169), (453, 455, 86), (455, 456, 171)]
for k in e:
    G.add_edge(k[0], k[1], weight=k[2])

# 普通的画图方式
#nx.draw(G, with_labels=True)

# 生成节点位置序列
pos = nx.spring_layout(G, iterations=200)

# 重新获取权重序列
weights = nx.get_edge_attributes(G, "weight")

# plt.figure(figsize=(150, 150), dpi=70)

# 画节点图, node_size默认是300
nx.draw_networkx(G, pos, with_labels=True, node_color="y",
                 font_size=8, node_size=200)

# 画权重图
# nx.draw_networkx_edge_labels(G, pos, edge_labels=weights, font_color="g")
nx.draw_networkx_edge_labels(
    G, pos, edge_labels=weights, font_color="k", font_size=6)

# 保存
plt.savefig("spanning_tree.png")
# plt.savefig("graph.png")

# 展示
plt.show()
