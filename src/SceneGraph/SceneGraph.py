from .SceneNode import SceneNode


class SceneGraph:
    def __init__(self, rootLink):
        self.root = SceneNode()
        self.constructNode(self.root, rootLink)

    def update(self):
        self.root.update()

    def getMesh(self):
        self.update()
        return self.root.getMesh()

    def constructNode(self, node, link):
        node.name = link.link.link_name
        node.joint = link.joint
        if node.joint != None:
            # Construct the joint node firstly; Deal with xyz and rpy of the node
            joint_xyz = node.joint.origin["xyz"]
            joint_rpy = node.joint.origin["rpy"]
            node.rotateXYZ(joint_rpy)
            node.translate(joint_xyz)
        # Construct the mesh nodes for multiple visuals in link
        visuals = link.link.visuals
        for visual in visuals:
            visual_node = SceneNode(node)
            node.addChild(visual_node)
            visual_node.name = node.name + "_mesh:" + visual.visual_name
            if visual.geometry_mesh["filename"] == None:
                raise RuntimeError("Invalid File path")
            visual_node.addMeshFile(visual.geometry_mesh["filename"])
            # Deal with xyz and rpy of the visual node
            visual_xyz = visual.origin['xyz']
            visual_rpy = visual.origin['rpy']
            visual_node.rotateXYZ(visual_rpy)
            visual_node.translate(visual_xyz)

        # Construct node for the children
        for child in link.children:
            child_node = SceneNode(node)
            node.addChild(child_node)
            self.constructNode(child_node, child)

