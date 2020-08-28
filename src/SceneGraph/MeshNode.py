import open3d as o3d
class MeshNode:
    def __init__(self, mesh = None):
        self.mesh = mesh

    def addMesh(self, mesh):
        if self.mesh == None:
            self.mesh = mesh
        else:
            self.mesh += mesh
        self.mesh.remove_unreferenced_vertices()

    def addMeshFile(self, mesh_file):
        # Read the mesh from obj file
        mesh = o3d.io.read_triangle_mesh(mesh_file)
        self.addMesh(mesh)