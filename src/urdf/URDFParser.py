import xml.etree.ElementTree as ET
import numpy as np
from Link import Link

def parseThreeNumber(string):
    strings = string.split(' ')
    numbers = np.array(list(map(float, strings)))
    return numbers

class URDFParser:
    def __init__(self, file_name):
        self.file_name = file_name
        self.links = {}

    # Parse the URDF(XML) file into a tree structure
    def parse(self):
        # Get the XML tree
        root_xml = ET.parse(self.file_name).getroot()
        self.links_xml = root_xml.findall("link")
        self.joints_xml = root_xml.findall("joint")
        # Parse links before parsing joints
        self.parseLinks()
        self.parseJoints()

    def parseLinks(self):
        for link_xml in self.links_xml:
            link_name = link_xml.attrib['name']
            link = Link(link_name)
            # Deal with the visual part
            visual_xml = link_xml.find("visual")
            if(visual_xml != None):
                # Get origin
                origin_xml = visual_xml.find('origin')
                if(origin_xml != None):
                    if('xyz' in origin_xml.attrib):
                        xyz = parseThreeNumber(origin_xml.attrib['xyz'])
                        link.setVisualOriginXyz(xyz)
                    if('rpy' in origin_xml.attrib):
                        rpy = parseThreeNumber(origin_xml.attrib['rpy'])
                        link.setVisualOriginRpy(rpy)
                # Get geometry
                geometry_xml = visual_xml.find('geometry')
                if(geometry_xml != None):
                    mesh_xml = geometry_xml.find('mesh')
                    if(mesh_xml != None):
                        filename = mesh_xml.attrib['filename']
                        link.setVisualGeometryMeshFilename(filename)
            self.links[link_name] = link

    def parseJoints(self):
        pass


if __name__ == "__main__":
    file_name = "../../data/43074/mobility.urdf"
    parser = URDFParser(file_name)
    parser.parse()

    for i in parser.links:
        print(parser.links[i])

