import bpy
import math
import struct
import bmesh
from mathutils import Vector, Euler

do_log = False

def my_log(str):
    if do_log:
        print(str)

def compare_connections(a, b, dir):
    dir_multiplier = Vector([-1 if x != 0 else 1 for x in dir])
    
    a_set = set(x.to_tuple(3) for x in a)
    b_set = set((dir_multiplier*x).to_tuple(3) for x in b)
    
    my_log(f"Comparing {a_set}, {b_set}")
    
    return a_set == b_set

def main():
    global do_log
    # Clear unused objects twice to free up duplicated names
    # we do these twice a the object references the mesh, so the second
    # purge will delete the mesh data
    bpy.data.orphans_purge()
    bpy.data.orphans_purge()
    
    print("\n\n\n\n\n==============================")
    if "tiles" not in bpy.data.collections:
        print("No tiles group to process")
        return
    if "tiles_rotatable" not in bpy.data.collections:
        print("No tiles_rotatable group to process")
        return
    
    export_objects = bpy.data.collections["tiles"].objects[:] + bpy.data.collections["tiles_rotatable"].objects[:]
    processessable_objects = bpy.data.collections["tiles"].objects[:]
    
    for obj in bpy.data.collections["tiles_rotatable"].objects:
        for rot in [0, 90, 180, 270]:
            new_obj = obj.copy()
            new_obj.name = f"{obj.name}|{rot}"
            rotation = Euler((0, 0, math.radians(rot)), 'XYZ').to_matrix().to_4x4()
            new_obj.data = new_obj.data.copy()
            new_obj.data.transform(rotation)
            
            processessable_objects.append(new_obj)
            
    
    
    objects = []
    for obj in processessable_objects:
        #if obj.name != "sand": continue
        directions = {
        ( 1, 0, 0) : [],
        (-1, 0, 0) : [],
        ( 0, 1, 0) : [],
        ( 0,-1, 0) : [],
        ( 0, 0, 1) : [],
        ( 0, 0,-1) : []}
        
        for vert in obj.data.vertices:
            for dir in directions:
                vec_dir = Vector(dir)
                if vec_dir.dot(vert.co) == 0.5:
                    directions[dir].append(vert.co)
    
        objects.append((obj, directions))
        
    # Check which objects can connect to each other in each direction
    obj_connections = []
    for obj_a in objects:
        directions = {
        ( 1, 0, 0) : [],
        (-1, 0, 0) : [],
        ( 0, 1, 0) : [],
        ( 0,-1, 0) : [],
        ( 0, 0, 1) : [],
        ( 0, 0,-1) : []}
            
        for obj_b in objects:
            print(f"connection {obj_a[0].name} and {obj_b[0].name}")
            for dir in directions:
                opposite_dir = tuple(-1*Vector(dir))
                
                obj_a_connections = obj_a[1][dir]
                obj_b_connections = obj_b[1][opposite_dir]
                
                if len(obj_a_connections) == 0 or len(obj_b_connections) == 0:
                    continue
                elif compare_connections(obj_a_connections, obj_b_connections, dir):
                    directions[dir].append(obj_b[0])
                    
        
        obj_connections.append((obj_a[0], directions))
  
    
    file_name = "tiles.bin"
    endianess = '<'
    file_header = b'TILE'
    file = open(file_name, 'wb')
    file.write(file_header)
    
    file.write(struct.pack(f"{endianess}I", len(export_objects)))
    
    for obj in export_objects:
        print(f"Exporting tile {obj.name}")
        obj_data = obj.data.copy()
        
        bm = bmesh.new()
        bm.from_mesh(obj_data)
        
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        
        bm.to_mesh(obj_data)
        bm.free()
        
        verts = []
        for face in obj_data.polygons:
            for index in face.vertices:
                verts.append(obj_data.vertices[index].co)
                
        file.write(struct.pack(f"{endianess}I", len(verts)))
        for vert in verts:
            file.write(struct.pack(f"{endianess}fffff", vert[0], vert[2], vert[1], 0, 0))    
        
    file.close()
        
    return


if __name__ == "__main__":
    main()