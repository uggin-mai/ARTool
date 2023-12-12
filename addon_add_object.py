# настройки аддона
bl_info = {
    "name": "ARTool",
    "author": "Evgenii Batulin, Tysyachniy Vladislav",
    "version": (0, 1),
    "blender": (2, 80, 0),
    "location": "View3D > N",
    "description": "Automatic Retopology Tool",
    "warning": "",
    "doc_url": "",
    "category": "",
}


import bpy
from bpy.types import (Panel, Operator)
from bpy.props import FloatVectorProperty
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from mathutils import Vector

import numpy as np
from itertools import product as all_combinations

# Операция по округлению значений позиционного поля на решётке
def lattice_op(p, o, n, target, scale, op = np.floor):
    t, d = np.cross(n, o), target - p
    return p + scale * (o * op(np.dot(o, d) / scale) + t * op(np.dot(t, d) / scale))

# Поиск точки пересечения касательных плоскостей
def intermediate_pos(p0, n0, p1, n1):
    n0p0, n0p1, n1p0, n1p1, n0n1 = np.dot(n0, p0), np.dot(n0, p1), np.dot(n1, p0), np.dot(n1, p1), np.dot(n0, n1)
    denom = 1.0 / (1.0 - n0n1*n0n1 + 1e-4)
    lambda_0 = 2.0*(n0p1 - n0p0 - n0n1*(n1p0 - n1p1))*denom
    lambda_1 = 2.0*(n1p0 - n1p1 - n0n1*(n0p1 - n0p0))*denom
    return 0.5 * (p0 + p1) - 0.25 * (n0 * lambda_0 + n1 * lambda_1)

# Поиск комбинации векторов с наибольшим углом
def compat_orientation_extrinsic(o0, n0, o1, n1):
    return max(all_combinations([o0, np.cross(n0, o0), -o0, -np.cross(n0, o0)], [o1, np.cross(n1, o1)]), key = lambda x: np.dot(x[0], x[1]))

# Поиск наилучшего смещения для заданных позиций ориентационного поля
def compat_position_extrinsic(o0, p0, n0, v0, o1, p1, n1, v1, scale):
    t0, t1, middle = np.cross(n0, o0), np.cross(n1, o1), intermediate_pos(v0, n0, v1, n1)
    p0, p1 = lattice_op(p0, o0, n0, middle, scale), lattice_op(p1, o1, n1, middle, scale)
    x = min(all_combinations([0, 1], [0, 1], [0, 1], [0, 1]),
        key = lambda x : np.linalg.norm((p0 + scale * (o0 * x[0] + t0 * x[1])) - (p1 + scale * (o1 * x[2] + t1 * x[3]))))
    result = (p0 + scale * (o0 * x[0] + t0 * x[1]), p1 + scale * (o1 * x[2] + t1 * x[3]))
    return result

class Mesh:
    def __init__(self, mesh):
        # Заполнение массивов данными модели
        self.nverts, self.nfaces, self.nedges = len(mesh.data.vertices), len(mesh.data.polygons), len(mesh.data.edges)
        self.vertices, self.faces, self.edges = np.zeros((self.nverts, 3)), np.zeros((self.nfaces, 3), np.uint32), np.zeros((self.nedges, 2))
        for i in range(self.nverts):
            self.vertices[i] = mesh.data.vertices[i].co
        for i in range(self.nfaces):
            self.faces[i] = mesh.data.polygons[i].vertices
        for i in range(self.nedges):
            self.edges[i] = mesh.data.edges[i].vertices
        self.edges = self.edges.astype(int)
        self.edges = self.edges.reshape(self.nedges, 2)
        # Вычисление нормалей вершин и граней
        v = [self.vertices[self.faces[:, i], :] for i in range(3)]
        face_normals = np.cross(v[2] - v[0], v[1] - v[0])
        face_normals /= np.linalg.norm(face_normals, axis=1)[:, None]
        self.normals = np.zeros((self.nverts, 3))
        for i, j in np.ndindex(self.faces.shape):
            self.normals[self.faces[i, j], :] += face_normals[i, :]
        self.normals /= np.linalg.norm(self.normals, axis=1)[:, None]
        # Вычисление матрицы смежности
        self.adjacency = [set() for _ in range(self.nfaces)]
        for i, j in np.ndindex(self.faces.shape):
            e0, e1 = self.faces[i, j], self.faces[i, (j+1)%3]
            self.adjacency[e0].add(e1)
            self.adjacency[e1].add(e0)
        # Случайная инициализация полей
        self.o_field = np.zeros((self.nverts, 3))
        self.p_field = np.zeros((self.nverts, 3))
        self.e_jumps = np.zeros((self.nedges, 4))
        min_pos, max_pos = self.vertices.min(axis=0), self.vertices.max(axis=0)
        np.random.seed(0)
        for i in range(self.nverts):
            d, p = np.random.standard_normal(3), np.random.random(3)
            d -= np.dot(d, self.normals[i]) * self.normals[i]
            self.o_field[i] = d / np.linalg.norm(d)
            self.p_field[i] = (1-p) * min_pos + p * max_pos

    # Оптимизация ориентационного поля
    def smooth_orientations(self, iterations = 10):
        for i in range(iterations):
            for i in np.random.permutation(np.arange(self.nverts)):
                o_i, n_i, weight = self.o_field[i], self.normals[i], 0
                for j in np.random.permutation(list(self.adjacency[i])):
                    o_compat = compat_orientation_extrinsic(o_i, n_i, self.o_field[j], self.normals[j])
                    o_i = weight*o_compat[0] + o_compat[1]
                    o_i -= n_i * np.dot(o_i, n_i)
                    o_i /= np.linalg.norm(o_i)
                    weight += 1
                self.o_field[i] = o_i

    # Оптимизация позиционного поля
    def smooth_positions(self, scale, iterations = 10):
        for i in range(iterations):
            print('Smoothing positions (%i/%i) ..' % (i+1, iterations))
            for i in np.random.permutation(np.arange(self.nverts)):
                o_i, p_i, n_i, v_i, weight = self.o_field[i], self.p_field[i], self.normals[i], self.vertices[i], 0
                for j in self.adjacency[i]:
                    p_compat = compat_position_extrinsic(o_i, p_i, n_i, v_i,
                        self.o_field[j], self.p_field[j], self.normals[j], self.vertices[j], scale)
                    p_i = (weight*p_compat[0] + p_compat[1]) / (weight + 1)
                    p_i -= n_i * np.dot(p_i - v_i, n_i)
                    weight += 1
                self.p_field[i] = lattice_op(p_i, o_i, n_i, v_i, scale, op = np.round)

    def save_position_field(self, mesh):
        for i in range(self.nverts):
            v1 = self.edges[i][0]
            v2 = self.edges[i][1]
            t1 = np.cross(self.normals[v1], self.o_field[v1])
            t2 = np.cross(self.normals[v2], self.o_field[v2])
            [mesh.data.vertices[i].co[0], mesh.data.vertices[i].co[1], mesh.data.vertices[i].co[2]] = self.p_field[i]

    def remove_unnecessary_edges(self, mesh):
        mesh.select_set(True)
        """
        for i in np.random.permutation(np.arange(self.nverts)):
            o_i, n_i, weight = self.o_field[i], self.normals[i], 0
            angles = []
            angles = np.array(angles)
            s = 0
            for j in self.adjacency[i]:
                edge = np.where((self.edges == (i,j)).all(axis=1))
                if any(edge):
                    v1 = self.vertices[i]
                    v2 = self.vertices[j]
                else:
                    edge = np.where((self.edges == (j,i)).all(axis=1))
                    v1 = self.vertices[i]
                    v2 = self.vertices[j]
                ev = v2 - v1
                el = np.linalg.norm(ev)
                ev = ev / el
                o_compat = compat_orientation_extrinsic(o_i, n_i, ev, self.normals[j]) #(self.normals[i]+self.normals[j])/2
                angles = np.append(el, [np.dot(o_compat[0], o_compat[1]) , edge[0][0]]) # / el
                s+=1
            angles = np.reshape(angles, (len(self.adjacency[i]), 2))
            angles = angles[angles[:,0].argsort()]
            for k in reversed(range(len(self.adjacency[i]) - 4)):
                mesh.data.edges[int(angles[k][1])].select = True
        """


class ButtonOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "retopology.1"
    bl_label = "Simple Object Operator"

    def execute(self, context):
        for obj in bpy.context.scene.objects:
            if obj.type == "MESH":
                mesh = Mesh(obj)
                mesh.smooth_orientations()
                mesh.smooth_positions(scale = 0.01)
                mesh.save_position_field(obj)
                mesh.remove_unnecessary_edges(obj)
        return {'FINISHED'}
    
class ARToolPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "ARTool Panel"
    bl_idname = "OBJECT_PT_retopology"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "ARTool"

    def draw(self, context):
        layout = self.layout
        obj = context.object
        row = layout.row()
        row.operator(ButtonOperator.bl_idname, text = 'Generate Retopologized Mesh', icon='MODIFIER')
    
from bpy.utils import register_class, unregister_class

_classes = [
    ButtonOperator,
    ARToolPanel
]

def register():
    for cls in _classes:
        register_class(cls)


def unregister():
    for cls in _classes:
        unregister_class(cls)


if __name__ == "__main__":
    register()
