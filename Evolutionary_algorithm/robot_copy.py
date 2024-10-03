import os
import sys
import networkx as nx
import numpy as np
import config
import copy
from joint import Joint
from new_link import BoxLink, SphereLink, CylinderLink

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)
from Parametrization.parametrization import *

class Robot:
    def __init__(self, graph=None):
        self.graph = graph if graph else nx.DiGraph()
        self.links = []
        self.joints = []
        self.decision_var = config.DECISION_VARIABLES
        self.fitness = None

    def initialize_from_graph(self):
        for node in self.graph.nodes(data=True):
            self.add_link(node[1]['type'], node[0], node[1]['name'], **node[1]['params'])
        
        for edge in self.graph.edges(data=True):
            self.add_joint(edge[2]['name'], edge[2]['type'], self.get_link_from_links(edge[0]), self.get_link_from_links(edge[1]), edge[2]['axis'])

        if 'mass' in self.decision_var:
            self.initialize_mass()
        if 'size' in self.decision_var:
            self.initialize_size()
        if 'com' in self.decision_var:
            self.initialize_com()

        self.create_links()
        self.updated_origin()
        for link in self.links:
            link.set_params(origin=link.params['origin'])

    def create_links(self):
        self.links = [create_link(link.unique_id, link.link_type, link.link_name, **link.params) for link in self.links]

    def get_link_from_links(self, link_name):
        for link in self.links:
            if link.link_name == link_name:
                return link
        return None

    def get_joint_from_joints(self, joint_name):
        for joint in self.joints:
            if joint.joint_name == joint_name:
                return joint
        return None

    def initialize_mass(self):
        for link in self.links:
            link.set_params(mass=np.random.uniform(config.MASS_RANGE[0], config.MASS_RANGE[1]))

    def initialize_com(self):
        for link in self.links:
            link.set_params(relative_com=np.random.uniform(0, 1))

    def initialize_size(self):
        for link in self.links:
            if isinstance(link, BoxLink):
                size = [np.random.uniform(config.SIZE_RANGE[0], config.SIZE_RANGE[1]) for _ in range(3)]
                link.set_params(size=size)
            elif isinstance(link, SphereLink):
                radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
                link.set_params(radius=radius)
            elif isinstance(link, CylinderLink):
                radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
                height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
                link.set_params(radius=radius, height=height)

    def add_link(self, shape_type, unique_id, link_name, **kwargs):
        if shape_type == "BoxLink":
            link = BoxLink(unique_id, link_name, **kwargs)
        elif shape_type == "SphereLink":
            link = SphereLink(unique_id, link_name, **kwargs)
        elif shape_type == "CylinderLink":
            link = CylinderLink(unique_id, link_name, **kwargs)
        self.links.append(link)

    def add_joint(self, joint_name, joint_type, parent, child, axis):
        joint = Joint(joint_name, joint_type, parent, child, axis=axis)
        self.joints.append(joint)

    def updated_origin(self):
        for link in self.links:
            parent_joint = next((joint for joint in self.joints if joint.child == link), None)
            if parent_joint:
                parent_link = parent_joint.parent
                if parent_link:
                    link_origin = [0, 0, 0]  # Default origin
                    if isinstance(link, BoxLink):
                        link_origin = [0, 0, -link.params['size'][2] / 2]
                    elif isinstance(link, CylinderLink):
                        link_origin = [0, 0, -link.params['height'] / 2]
                    elif isinstance(link, SphereLink):
                        link_origin = [0, 0, -link.params['radius']]
                    link.set_params(origin=link_origin)
                    parent_joint.set_joint_pos([parent_link.params['size'][0] / 2, 0, -parent_link.params['size'][2] / 2])

    def updated_inertia_com(self):
        for link in self.links:
            inertia = Inertia(
                rho=link.params.get('rho', 0),
                r=link.params.get('radius', 0),
                h=link.params.get('height', 0),
                x=link.params.get('size', [0, 0, 0])[0],
                y=link.params.get('size', [0, 0, 0])[1],
                z=link.params.get('size', [0, 0, 0])[2],
                type=link.link_type,
                m=link.params.get('mass', 0)
            )
            position = Com_position(
                r=link.params.get('radius', 0),
                h=link.params.get('height', 0),
                x=link.params.get('size', [0, 0, 0])[0],
                y=link.params.get('size', [0, 0, 0])[1],
                z=link.params.get('size', [0, 0, 0])[2],
                type=link.link_type,
                relative_com=link.params.get('relative_com', 0.5)
            )
            Ixx, Iyy, Izz = updated_inertia_com(link.params['relative_com'], inertia, position)
            link.params['inertia'] = [Ixx, 0, 0, Iyy, 0, Izz]
            link.params['com'] = position.com_pos

    def updated_params(self):
        self.updated_inertia_com()
        self.updated_origin()

    def calculate_fitness(self):
        return sum(link.params['mass'] for link in self.links if 'mass' in link.params)

    @property
    def get_fitness(self):
        if self.fitness is None:
            self.fitness = self.calculate_fitness()
        return self.fitness

    def copy(self):
        return copy.deepcopy(self)

# Example usage
if __name__ == "__main__":
    G = nx.DiGraph()
    G.add_node(1, type="BoxLink", name="base", params={'size': [0.01, 0.01, 0.01], 'mass': 0.01})
    G.add_node(2, type="BoxLink", name="trunk", params={'size': [1.0, 0.5, 0.5], 'mass': 5.0})

    # Add limbs dynamically based on some morphology definition
    limb_count = 4
    articulations_per_limb = 2  # You can make this dynamic as well
    for i in range(limb_count):
        for j in range(articulations_per_limb):
            link_id = 3 + i * articulations_per_limb + j
            G.add_node(link_id, type="CylinderLink", name=f"Limb_{i}_articulation_{j}", params={'radius': 0.1, 'height': 0.5, 'mass': 1.0})

    G.add_edge(1, 2, name="floating_base", type="fixed", axis=[0, 0, 0])
    for i in range(limb_count):
        for j in range(articulations_per_limb):
            parent_id = 2 if j == 0 else 3 + i * articulations_per_limb + j - 1
            child_id = 3 + i * articulations_per_limb + j
            G.add_edge(parent_id, child_id, name=f"joint_{parent_id}_{child_id}", type="revolute", axis=[1, 0, 0])

    robot = Robot(graph=G)
    robot.initialize_from_graph()

    print(robot.links)
    print(robot.joints)
