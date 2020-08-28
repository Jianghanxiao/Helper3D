class URDFTree:
    # Construct the URDF tree based on the parser
    def __init__(self, links, joints):
        self.links = links
        self.joints = joints