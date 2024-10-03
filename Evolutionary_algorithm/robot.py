import os, sys
from joint import Joint
from new_link import BoxLink, SphereLink, CylinderLink
import numpy as np
import config
import copy


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)
from Parametrization.parametrization import *

class Robot:
    def __init__(self, morphology="quadruped"):
        self.morphology = morphology
        self.links = []
        self.joints = []
        self.decision_var = config.DECISION_VARIABLES
        self.fitness = None

    def initialize_fixed_morphology(self):
        # define different scenarios for the initialization of the robots

        # Add links
        self.add_link("BoxLink", unique_id=1, link_name="base")
        self.add_link("BoxLink", unique_id=2, link_name="trunk")
        
        # Front right limb
        self.add_link("SphereLink", unique_id=3, link_name="FR_hip")
        self.add_link("CylinderLink", unique_id=4, link_name="FR_thigh")
        self.add_link("SphereLink", unique_id=19, link_name="FR_knee")
        self.add_link("CylinderLink", unique_id=5, link_name="FR_calf")
        self.add_link("SphereLink", unique_id=6, link_name="FR_foot")
        
        # Front left limb
        self.add_link("SphereLink", unique_id=7, link_name="FL_hip")
        self.add_link("CylinderLink", unique_id=8, link_name="FL_thigh")
        self.add_link("SphereLink", unique_id=20, link_name="FL_knee")
        self.add_link("CylinderLink", unique_id=9, link_name="FL_calf")
        self.add_link("SphereLink", unique_id=10, link_name="FL_foot")
        
        # Rear right limb
        self.add_link("SphereLink", unique_id=11, link_name="RR_hip")
        self.add_link("CylinderLink", unique_id=12, link_name="RR_thigh")
        self.add_link("SphereLink", unique_id=21, link_name="RR_knee")
        self.add_link("CylinderLink", unique_id=13, link_name="RR_calf")
        self.add_link("SphereLink", unique_id=14, link_name="RR_foot")
        
        # Rear left limb
        self.add_link("SphereLink", unique_id=15, link_name="RL_hip")
        self.add_link("CylinderLink", unique_id=16, link_name="RL_thigh")
        self.add_link("SphereLink", unique_id=22, link_name="RL_knee")
        self.add_link("CylinderLink", unique_id=17, link_name="RL_calf")
        self.add_link("SphereLink", unique_id=18, link_name="RL_foot")

        # Add joints
        self.add_joint(joint_name="floating_base", joint_type="fixed", parent=self.get_link_from_links("base"), child=self.get_link_from_links("trunk"), axis=[0, 0, 0])
        self.add_joint(joint_name="FL_hip_joint", joint_type="revolute", parent=self.get_link_from_links("trunk"), child=self.get_link_from_links("FL_hip"), axis=[1, 0, 0])
        self.add_joint(joint_name="FR_hip_joint", joint_type="revolute", parent=self.get_link_from_links("trunk"), child=self.get_link_from_links("FR_hip"), axis=[1, 0, 0])
        self.add_joint(joint_name="RL_hip_joint", joint_type="revolute", parent=self.get_link_from_links("trunk"), child=self.get_link_from_links("RL_hip"), axis=[1, 0, 0])
        self.add_joint(joint_name="RR_hip_joint", joint_type="revolute", parent=self.get_link_from_links("trunk"), child=self.get_link_from_links("RR_hip"), axis=[1, 0, 0])

        self.add_joint(joint_name="FL_thigh_joint", joint_type="revolute", parent=self.get_link_from_links("FL_hip"), child=self.get_link_from_links("FL_thigh"), axis=[0, 1, 0])
        self.add_joint(joint_name="FR_thigh_joint", joint_type="revolute", parent=self.get_link_from_links("FR_hip"), child=self.get_link_from_links("FR_thigh"), axis=[0, 1, 0])
        self.add_joint(joint_name="RL_thigh_joint", joint_type="revolute", parent=self.get_link_from_links("RL_hip"), child=self.get_link_from_links("RL_thigh"), axis=[0, 1, 0])
        self.add_joint(joint_name="RR_thigh_joint", joint_type="revolute", parent=self.get_link_from_links("RR_hip"), child=self.get_link_from_links("RR_thigh"), axis=[0, 1, 0])

        self.add_joint(joint_name="FL_knee_joint", joint_type="fixed", parent=self.get_link_from_links("FL_thigh"), child=self.get_link_from_links("FL_knee"), axis=[0, 0, 0])
        self.add_joint(joint_name="FR_knee_joint", joint_type="fixed", parent=self.get_link_from_links("FR_thigh"), child=self.get_link_from_links("FR_knee"), axis=[0, 0, 0])
        self.add_joint(joint_name="RL_knee_joint", joint_type="fixed", parent=self.get_link_from_links("RL_thigh"), child=self.get_link_from_links("RL_knee"), axis=[0, 0, 0])
        self.add_joint(joint_name="RR_knee_joint", joint_type="fixed", parent=self.get_link_from_links("RR_thigh"), child=self.get_link_from_links("RR_knee"), axis=[0, 0, 0])

        self.add_joint(joint_name="FL_calf_joint", joint_type="revolute", parent=self.get_link_from_links("FL_knee"), child=self.get_link_from_links("FL_calf"), axis=[0, 1, 0])
        self.add_joint(joint_name="FR_calf_joint", joint_type="revolute", parent=self.get_link_from_links("FR_knee"), child=self.get_link_from_links("FR_calf"), axis=[0, 1, 0])
        self.add_joint(joint_name="RL_calf_joint", joint_type="revolute", parent=self.get_link_from_links("RL_knee"), child=self.get_link_from_links("RL_calf"), axis=[0, 1, 0])
        self.add_joint(joint_name="RR_calf_joint", joint_type="revolute", parent=self.get_link_from_links("RR_knee"), child=self.get_link_from_links("RR_calf"), axis=[0, 1, 0])

        self.add_joint(joint_name="FL_foot_joint", joint_type="fixed", parent=self.get_link_from_links("FL_calf"), child=self.get_link_from_links("FL_foot"), axis=[0, 0, 0])
        self.add_joint(joint_name="FR_foot_joint", joint_type="fixed", parent=self.get_link_from_links("FR_calf"), child=self.get_link_from_links("FR_foot"), axis=[0, 0, 0])
        self.add_joint(joint_name="RL_foot_joint", joint_type="fixed", parent=self.get_link_from_links("RL_calf"), child=self.get_link_from_links("RL_foot"), axis=[0, 0, 0])
        self.add_joint(joint_name="RR_foot_joint", joint_type="fixed", parent=self.get_link_from_links("RR_calf"), child=self.get_link_from_links("RR_foot"), axis=[0, 0, 0])

        if 'mass' in self.decision_var:
            self.initialize_fixed_morphology_mass()
        if 'size' in self.decision_var:
            self.initialize_fixed_morphology_size()
        if 'com' in self.decision_var:
            self.initialize_fixed_morphology_com()

        # using create_link function from parametrization.py
        self.create_links()

        # updating origin of the links
        self.updated_origin()
        for i in range(len(self.links)):
            self.links[i].set_params(origin=self.links[i].params['origin'])

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

    def initialize_fixed_morphology_mass(self):  
        for i in range(len(self.links)): 
            self.links[i].set_params(mass=np.random.uniform(config.MASS_RANGE[0], config.MASS_RANGE[1]))

    def initialize_fixed_morphology_com(self):
        for i in range(len(self.links)):
            self.links[i].set_params(relative_com=np.random.uniform(0, 1))

    def initialize_fixed_morphology_size(self):

        trunk_height = np.random.uniform(config.SIZE_RANGE_HEIGHT[0], config.SIZE_RANGE_HEIGHT[1])
        trunk_width = np.random.uniform(config.SIZE_RANGE_WIDTH[0], config.SIZE_RANGE_WIDTH[1])
        trunk_length = np.random.uniform(config.SIZE_RANGE_LENGTH[0], config.SIZE_RANGE_LENGTH[1])
        trunk_size = [trunk_length, trunk_width, trunk_height]
        front_thigh_height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
        front_thigh_radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
        front_calf_height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
        front_calf_radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
        rear_thigh_height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
        rear_thigh_radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
        rear_calf_height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
        rear_calf_radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])

        base_size = [0.01, 0.01, 0.01]
        foot_sphere_radius = 0.7*min(rear_calf_radius, front_calf_radius)

        # Update the size of the links
        for i in range(len(self.links)):

            link_name = self.links[i].link_name

            if isinstance(self.links[i], BoxLink):
                if self.links[i].link_name == 'base':
                    self.links[i].set_params(size=base_size)
                elif self.links[i].link_name == 'trunk':
                    self.links[i].set_params(size=trunk_size)

            elif isinstance(self.links[i], SphereLink):
                if ('FR' in link_name or 'FL' in link_name) and 'hip' in link_name:
                    self.links[i].set_params(radius=front_thigh_radius)
                elif ('RR' in link_name or 'RL' in link_name) and 'hip' in link_name:
                    self.links[i].set_params(radius=rear_thigh_radius)
                elif ('FR' in link_name or 'FL' in link_name) and 'knee' in link_name:
                    self.links[i].set_params(radius=max(front_calf_radius, front_thigh_radius))
                elif ('RR' in link_name or 'RL' in link_name) and 'knee' in link_name:
                    self.links[i].set_params(radius=max(rear_calf_radius, rear_thigh_radius))
                elif 'foot' in link_name:
                    self.links[i].set_params(radius=foot_sphere_radius)

            elif isinstance(self.links[i], CylinderLink):
                if ('FR' in link_name or 'FL' in link_name) and 'thigh' in link_name:
                    self.links[i].set_params(radius=front_thigh_radius, height=front_thigh_height)
                elif ('FR' in link_name or 'FL' in link_name) and 'calf' in link_name:
                    self.links[i].set_params(radius=front_calf_radius, height=front_calf_height)
                elif ('RR' in link_name or 'RL' in link_name) and 'thigh' in link_name:
                    self.links[i].set_params(radius=rear_thigh_radius, height=rear_thigh_height)
                elif ('RR' in link_name or 'RL' in link_name) and 'calf' in link_name:
                    self.links[i].set_params(radius=rear_calf_radius, height=rear_calf_height)

    # def initialize_fixed_morphology_size(self):
    #     for i in range(len(self.links)):
    #         if isinstance(self.links[i], BoxLink):
    #             self.links[i].set_params(size=[np.random.uniform(config.SIZE_RANGE[0], config.SIZE_RANGE[1]) for _ in range(3)])
    #         elif isinstance(self.links[i], SphereLink):
    #             self.links[i].set_params(radius=np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1]))
    #         elif isinstance(self.links[i], CylinderLink):
    #             self.links[i].set_params(radius=np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1]))
    #             self.links[i].set_params(height=np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1]))

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

    def updated_origin_rear(self, base, rear_thigh, rear_calf):
        # Update links origin
        self.get_link_from_links("RR_thigh").set_params(origin=[0, 0, -rear_thigh.params['height']/2])
        self.get_link_from_links("RL_thigh").set_params(origin=[0, 0, -rear_thigh.params['height']/2])
        self.get_link_from_links("RR_hip").set_params(origin=[0, 0, 0])
        self.get_link_from_links("RL_hip").set_params(origin=[0, 0, 0])
        self.get_link_from_links("RR_calf").set_params(origin=[0, 0, -rear_calf.params['height']/2])
        self.get_link_from_links("RL_calf").set_params(origin=[0, 0, -rear_calf.params['height']/2])
        self.get_link_from_links("RR_knee").set_params(origin=[0, 0, 0])
        self.get_link_from_links("RL_knee").set_params(origin=[0, 0, 0])
        self.get_link_from_links("RR_foot").set_params(origin=[0, 0, 0])
        self.get_link_from_links("RL_foot").set_params(origin=[0, 0, 0])

        # Update joints position
        self.get_joint_from_joints("RR_hip_joint").set_joint_pos([-base.params['size'][0]/2, -base.params['size'][1]/2, -0.8*base.params['size'][2]/2])
        self.get_joint_from_joints("RL_hip_joint").set_joint_pos([-base.params['size'][0]/2, base.params['size'][1]/2, -0.8*base.params['size'][2]/2])
        self.get_joint_from_joints("RR_thigh_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("RL_thigh_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("RR_knee_joint").set_joint_pos([0, 0, -rear_thigh.params['height']])
        self.get_joint_from_joints("RL_knee_joint").set_joint_pos([0, 0, -rear_thigh.params['height']])
        self.get_joint_from_joints("RR_calf_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("RL_calf_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("RR_foot_joint").set_joint_pos([0, 0, -rear_calf.params['height']])
        self.get_joint_from_joints("RL_foot_joint").set_joint_pos([0, 0, -rear_calf.params['height']])

    def updated_origin_front(self, base, front_thigh, front_calf):
        # Update links origin
        self.get_link_from_links("FR_thigh").set_params(origin=[0, 0, -front_thigh.params['height']/2])
        self.get_link_from_links("FL_thigh").set_params(origin=[0, 0, -front_thigh.params['height']/2])
        self.get_link_from_links("FR_hip").set_params(origin=[0, 0, 0])
        self.get_link_from_links("FL_hip").set_params(origin=[0, 0, 0])
        self.get_link_from_links("FR_calf").set_params(origin=[0, 0, -front_calf.params['height']/2])
        self.get_link_from_links("FL_calf").set_params(origin=[0, 0, -front_calf.params['height']/2])
        self.get_link_from_links("FR_knee").set_params(origin=[0, 0, 0])
        self.get_link_from_links("FL_knee").set_params(origin=[0, 0, 0])
        self.get_link_from_links("FR_foot").set_params(origin=[0, 0, 0])
        self.get_link_from_links("FL_foot").set_params(origin=[0, 0, 0])

        # Update joints position
        self.get_joint_from_joints("FR_hip_joint").set_joint_pos([base.params['size'][0]/2, -base.params['size'][1]/2, -0.8*base.params['size'][2]/2])
        self.get_joint_from_joints("FL_hip_joint").set_joint_pos([base.params['size'][0]/2, base.params['size'][1]/2, -0.8*base.params['size'][2]/2])
        self.get_joint_from_joints("FR_thigh_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("FL_thigh_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("FR_knee_joint").set_joint_pos([0, 0, -front_thigh.params['height']])
        self.get_joint_from_joints("FL_knee_joint").set_joint_pos([0, 0, -front_thigh.params['height']])
        self.get_joint_from_joints("FR_calf_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("FL_calf_joint").set_joint_pos([0, 0, 0])
        self.get_joint_from_joints("FR_foot_joint").set_joint_pos([0, 0, -front_calf.params['height']])
        self.get_joint_from_joints("FL_foot_joint").set_joint_pos([0, 0, -front_calf.params['height']])

    def updated_origin(self):
        base = self.get_link_from_links("trunk")
        # Since the morphology is symmetrical, the origin of left and right limbs can be updated simultaneously
        front_thigh = self.get_link_from_links("FR_thigh")
        front_calf = self.get_link_from_links("FR_calf")
        rear_thigh = self.get_link_from_links("RR_thigh")
        rear_calf = self.get_link_from_links("RR_calf")
        self.updated_origin_front(base, front_thigh, front_calf)
        self.updated_origin_rear(base, rear_thigh, rear_calf)

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
                relative_com=link.params.get('relative_com')
            )
            Ixx, Iyy, Izz = updated_inertia_com(link.params['relative_com'], inertia, position)
            link.params['inertia'] = [Ixx, 0, 0, Iyy, 0, Izz]
            link.params['com'] = position.com_pos
        
    def updated_params(self):
        self.updated_inertia_com()
        self.updated_origin()

    def create_link_joint(parent_link, child_link, joint_type, joint_name, axis):
        pass

    def calculate_fitness(self):
        # example is the sum of the mass of all links
        return sum([link.params['mass'] for link in self.links])
    
    @property
    def get_fitness(self):
        if self.fitness is None:
            self.fitness = self.calculate_fitness()
        return self.fitness
    
    def copy(self):
        return copy.deepcopy(self)

# Example usage
if __name__ == "__main__":
    robot = Robot(morphology='quadruped')

