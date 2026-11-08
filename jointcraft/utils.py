"""Shared Blender adapters used by fixed joints and hinges."""
import bpy
from mathutils import Vector


def mm_to_bu(context, mm):
    scale = context.scene.unit_settings.scale_length or 1.0
    return mm * .001 / scale


def bounds_world(obj):
    points = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    return ([min(p[i] for p in points) for i in range(3)],
            [max(p[i] for p in points) for i in range(3)])


def mesh_in_scene(scene, obj):
    return obj.type == 'MESH' and scene.objects.get(obj.name) == obj


def boolean_mesh(context, target, cutter, operation):
    modifier = target.modifiers.new('JC temporary ' + operation, 'BOOLEAN')
    modifier.operation, modifier.solver, modifier.object = operation, 'EXACT', cutter
    try:
        depsgraph = context.evaluated_depsgraph_get()
        evaluated = target.evaluated_get(depsgraph)
        result = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True,
                                                depsgraph=depsgraph)
        if result is None or not result.vertices or not result.polygons:
            if result is not None:
                bpy.data.meshes.remove(result)
            raise RuntimeError('Boolean evaluation failed')
        return result
    finally:
        target.modifiers.remove(modifier)
