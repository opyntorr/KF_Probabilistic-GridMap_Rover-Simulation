import open3d as o3d
import os

in_path = "/home/opyntorr/agv_uav_project_jetauto_Vilchis/src/mi_proyecto_sim/models/laberinto_real/meshes/jaula.stl"
out_path = "/home/opyntorr/agv_uav_project_jetauto_Vilchis/src/mi_proyecto_sim/models/laberinto_real/meshes/jaula_opt.stl"

mesh = o3d.io.read_triangle_mesh(in_path)
target_triangles = max(1000, len(mesh.triangles) // 10)
mesh_opt = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)

mesh_opt.compute_vertex_normals()
o3d.io.write_triangle_mesh(out_path, mesh_opt)
print("Done writing with normals!")
