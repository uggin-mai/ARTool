# настройки аддона
bl_info = {
    "name": "ARToolC",
    "author": "Evgenii Batulin, Tysyachniy Vladislav",
    "version": (0, 1),
    "blender": (2, 80, 0),
    "location": "View3D > N",
    "description": "Automatic Retopology Tool (Compiled)",
    "warning": "",
    "doc_url": "",
    "category": "",
}


import bpy
import shutil
import tempfile
import subprocess
import os
from bpy.types import AddonPreferences;
from bpy.types import (Panel, Operator)
from bpy.props import FloatVectorProperty
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from mathutils import Vector

import numpy as np
from itertools import product as all_combinations

class ButtonOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "retopology.1"
    bl_label = "Simple Object Operator"

    exported = False

    loc = None
    rot = None
    scl = None
    meshname = None

    def execute(self, context):
        directory = os.path.dirname(os.path.abspath(__file__))

        dirname = os.path.join(directory,'art_temp')
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        orig = os.path.join(dirname,'original.obj')
        output = os.path.join(dirname,'out.obj')

        mesh = bpy.data.objects[self.meshname]
        mesh.hide_viewport = False
        options = ['-o', output]

        cmd = [os.path.dirname(os.path.abspath(__file__)) + "/art.exe"] + options + [orig]
        print(cmd)

        subprocess.run(cmd)

        bpy.ops.import_scene.obj(filepath=output,
                                 use_split_objects=False,
                                 use_smooth_groups=False,
                                 use_image_search=False,
                                 axis_forward='-Z', axis_up='Y')
        imported_mesh = bpy.context.selected_objects[0]
        imported_mesh.name = mesh.name + '_art'
        for i in mesh.data.materials:
            print('setting mat: ' + i.name)
            imported_mesh.data.materials.append(i)
        for edge in imported_mesh.data.edges:
            edge.use_edge_sharp = False
        for other_obj in bpy.data.objects:
            other_obj.select_set(state=False)
        imported_mesh.select_set(state=True)
        imported_mesh.active_material.use_nodes = False
        imported_mesh.data.use_auto_smooth = False

        bpy.ops.object.shade_flat()
        bpy.ops.mesh.customdata_custom_splitnormals_clear()

        mesh.select_set(state=True)
        bpy.context.view_layer.objects.active = mesh
        bpy.ops.object.data_transfer(use_reverse_transfer=False,
                                     use_freeze=False, data_type='UV', use_create=True, vert_mapping='NEAREST',
                                     edge_mapping='NEAREST', loop_mapping='NEAREST_POLYNOR', poly_mapping='NEAREST',
                                     use_auto_transform=False, use_object_transform=True, use_max_distance=False,
                                     max_distance=1.0, ray_radius=0.0, islands_precision=0.1, layers_select_src='ACTIVE',
                                     layers_select_dst='ACTIVE', mix_mode='REPLACE', mix_factor=1.0)
        mesh.select_set(state=False)
        mesh.hide_viewport = True
        imported_mesh.select_set(state=False)
        os.remove(output)

        return {'FINISHED'}
        

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width = 400)
    
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
