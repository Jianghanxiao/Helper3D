import trimesh

class MeshNode:
    def __init__(self):
        self.mesh = None

    def addMesh(self, mesh):
        if self.mesh == None:
            self.mesh = mesh
        else:
            self.mesh = trimesh.scene.scene.append_scenes([self.mesh, mesh])

    def addMeshFile(self, mesh_file):
        # Read the mesh from obj file
        mesh = trimesh.load(mesh_file)
        if not isinstance(mesh, trimesh.Scene):
            scene = trimesh.Scene()
            scene.add_geometry(mesh)
            mesh = scene
        self.addMesh(mesh)

    def getMesh(self, worldMatrix):
        if self.mesh == None:
            return None    
        new_mesh = self.mesh.copy()
        new_mesh.apply_transform(worldMatrix)
        return new_mesh