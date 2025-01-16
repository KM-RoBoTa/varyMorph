import os, sys
from utils.joint import Joint
from utils.link import BoxLink, SphereLink, CylinderLink
import numpy as np
import config


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)
from parametrization.parametrization import *

# Dictionary mapping morphology names to the number of legs
morphology_to_legs = {
    "biped": 2,
    "tripod": 3,
    "quadruped": 4,
    "pentapod": 5,
    "hexapod": 6,
}

class Robot:
    def __init__(self):
        self.morphology = config.MORPHOLOGY
        self.num_legs = morphology_to_legs.get(config.MORPHOLOGY)
        self.links = []
        self.joints = []
        self.fitness = None

    def initialize_parametrized_morphology(self):
        """
        Create a robot with a variable number of legs (2 <= num_legs <= 6).
        We will do the following steps:
          1) Create the trunk/base
          2) Create each leg in a loop
          3) Create the random parameters (mass, size, com, etc.)
          4) Update link geometry and joint positions
        """
        # Always start with the base + trunk
        self.add_link("BoxLink", unique_id=1, link_name="base")
        self.add_link("BoxLink", unique_id=2, link_name="trunk")

        # Create each leg
        #    For each leg, we will add something like:
        #       (a) hip sphere
        #       (b) thigh cylinder
        #       (c) knee sphere
        #       (d) calf cylinder
        #       (e) foot sphere
        #    Then connect them with joints.
        current_unique_id = 3           # track unique_id so we don't conflict
        self.add_joint(joint_name="floating_base", joint_type="fixed", parent=self.get_link_from_links("base"), child=self.get_link_from_links("trunk"), axis=[0, 0, 0])
        
        if self.num_legs not in [2, 3, 4, 5, 6]:
            raise ValueError(f"Unsupported morphology: {self.morphology}")
        
        for leg_id in range(self.num_legs):
            # 1) Create the link names
            hip_name    = f"leg_{leg_id}_hip"
            thigh_name  = f"leg_{leg_id}_thigh"
            knee_name   = f"leg_{leg_id}_knee"
            calf_name   = f"leg_{leg_id}_calf"
            foot_name   = f"leg_{leg_id}_foot"

            # 2) Add the links for each leg
            self.add_link("SphereLink",   current_unique_id,   link_name=hip_name)
            current_unique_id += 1
            self.add_link("CylinderLink", current_unique_id,   link_name=thigh_name)
            current_unique_id += 1
            self.add_link("SphereLink",   current_unique_id,   link_name=knee_name)
            current_unique_id += 1
            self.add_link("CylinderLink", current_unique_id,   link_name=calf_name)
            current_unique_id += 1
            self.add_link("SphereLink",   current_unique_id,   link_name=foot_name)
            current_unique_id += 1

            # 3) Add the joints for each leg
            #      We connect:
            #         trunk -> hip
            #         hip -> thigh
            #         thigh -> knee
            #         knee -> calf
            #         calf -> foot
            #      For axis, we keep them the same as your quadruped example or vary them as needed
            trunk_link  = self.get_link_from_links("trunk")
            hip_link    = self.get_link_from_links(hip_name)
            thigh_link  = self.get_link_from_links(thigh_name)
            knee_link   = self.get_link_from_links(knee_name)
            calf_link   = self.get_link_from_links(calf_name)
            foot_link   = self.get_link_from_links(foot_name)

            # trunk -> hip
            self.add_joint(
                joint_name=f"leg_{leg_id}_hip_joint",
                joint_type="revolute",
                parent=trunk_link,
                child=hip_link,
                axis=[1, 0, 0]
            )
            # hip -> thigh
            self.add_joint(
                joint_name=f"leg_{leg_id}_thigh_joint",
                joint_type="revolute",
                parent=hip_link,
                child=thigh_link,
                axis=[0, 1, 0]
            )
            # thigh -> knee
            self.add_joint(
                joint_name=f"leg_{leg_id}_knee_joint",
                joint_type="fixed",
                parent=thigh_link,
                child=knee_link,
                axis=[0, 0, 0]
            )
            # knee -> calf
            self.add_joint(
                joint_name=f"leg_{leg_id}_calf_joint",
                joint_type="revolute",
                parent=knee_link,
                child=calf_link,
                axis=[0, 1, 0]
            )
            # calf -> foot
            self.add_joint(
                joint_name=f"leg_{leg_id}_foot_joint",
                joint_type="fixed",
                parent=calf_link,
                child=foot_link,
                axis=[0, 0, 0]
            )

        self.initialize_fixed_morphology_mass()
        self.initialize_fixed_morphology_size()

        # Create the links' properties
        self.create_links()
        self.initialize_fixed_morphology_com()

        # updating origin and com of the links
        self.updated_params()

    def get_hip_positions(self, base_params):
        """
        Returns a dictionary that maps a 'leg name' -> [x, y, z] position
        for the hip joint on the trunk.
        """
        num_legs = self.num_legs

        length = base_params['size'][0]
        width  = base_params['size'][1]
        height = base_params['size'][2]
        
        x_half = length / 2.0
        y_half = width  / 2.0
        
        z_offset = -0.8 * (height / 2.0)

        positions = {}

        if num_legs == 2:
            # Biped
            positions["leg_0"] = [-x_half, 0.0, z_offset]
            positions["leg_1"] = [x_half,  0.0, z_offset]

        elif num_legs == 3:
            # Tipod
            positions["leg_0"] = [ x_half, -y_half, z_offset]  # front-right corner
            positions["leg_1"] = [ x_half,  y_half, z_offset]  # front-left corner
            positions["leg_2"] = [-x_half,     0.0,  z_offset] # centerline on opposite side

        elif num_legs == 4:
            # Quadruped
            positions["leg_0"] = [ x_half, -y_half, z_offset]   # front-right
            positions["leg_1"] = [ x_half,  y_half, z_offset]   # front-left
            positions["leg_2"] = [-x_half, -y_half, z_offset]   # rear-right
            positions["leg_3"] = [-x_half,  y_half, z_offset]   # rear-left

        elif num_legs == 5:
            # Pentaped
            positions["leg_0"] = [ x_half,  y_half,   z_offset]  # corner
            positions["leg_1"] = [ x_half, -y_half,   z_offset]  # corner
            positions["leg_2"] = [    0.0,    y_half, z_offset]  # center of length side
            positions["leg_3"] = [    0.0,   -y_half, z_offset]  # center of other length side
            positions["leg_4"] = [-x_half,    0.0,    z_offset]  # center of opposite width

        elif num_legs == 6:
            # Hexapod
            positions["leg_0"] = [ x_half, -y_half, z_offset]
            positions["leg_1"] = [ x_half,  y_half, z_offset]
            positions["leg_2"] = [    0.0, -y_half, z_offset]   # middle-right
            positions["leg_3"] = [    0.0,  y_half, z_offset]   # middle-left
            positions["leg_4"] = [-x_half, -y_half, z_offset]
            positions["leg_5"] = [-x_half,  y_half, z_offset]

        else:
            # Default or error case
            raise ValueError(f"Unsupported number of legs: {num_legs}")

        return positions
    
    def updated_origin(self):
        trunk = self.get_link_from_links("trunk")
        base_params = trunk.params

        hip_positions = self.get_hip_positions(base_params)

        for leg, position in hip_positions.items():
            # e.g. "leg_0"
            leg_id = leg

            # Access link names
            hip_link_name    = f"{leg_id}_hip"
            thigh_link_name  = f"{leg_id}_thigh"
            knee_link_name   = f"{leg_id}_knee"
            calf_link_name   = f"{leg_id}_calf"
            foot_link_name   = f"{leg_id}_foot"

            # Access joint names
            hip_joint_name   = f"{leg_id}_hip_joint"
            thigh_joint_name = f"{leg_id}_thigh_joint"
            knee_joint_name  = f"{leg_id}_knee_joint"
            calf_joint_name  = f"{leg_id}_calf_joint"
            foot_joint_name  = f"{leg_id}_foot_joint"

            # Access link
            thigh_link = self.get_link_from_links(thigh_link_name)
            calf_link  = self.get_link_from_links(calf_link_name)

            # Set params for links
            thigh_link.set_params(origin=[0, 0, -thigh_link.params['height']/2])
            calf_link.set_params(origin=[0, 0, -calf_link.params['height']/2])

            # Set the joints positions
            hip_joint = self.get_joint_from_joints(hip_joint_name)
            hip_joint.set_joint_pos(position)

            thigh_joint = self.get_joint_from_joints(thigh_joint_name)
            thigh_joint.set_joint_pos([0, 0, 0])  # same logic you had

            knee_joint = self.get_joint_from_joints(knee_joint_name)
            knee_joint.set_joint_pos([0, 0, -thigh_link.params['height']])

            calf_joint = self.get_joint_from_joints(calf_joint_name)
            calf_joint.set_joint_pos([0, 0, 0])

            foot_joint = self.get_joint_from_joints(foot_joint_name)
            foot_joint.set_joint_pos([0, 0, -calf_link.params['height']])

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
    
    def assign_leg_dimensions(self, num_legs, mode):
        """
        Return a dict: leg_index -> {thigh_height, thigh_radius, calf_height, calf_radius, foot_radius}
        based on 'mode' which can be:
        - "random"        : each leg fully randomized
        - "symmetrical"   : all legs share the same dims
        - "bilateral" : certain legs share dimensions in pairs or solos
        """

        leg_dims = {}

        def random_leg_dims():
            # You can adapt these random calls to your config.* constants
            thigh_height = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
            thigh_radius = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
            calf_height  = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])
            calf_radius  = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
            # foot radius: example 0.7 * min(thigh_radius, calf_radius)
            foot_radius  = 0.7 * min(thigh_radius, calf_radius)
            return {
                "thigh_height": thigh_height,
                "thigh_radius": thigh_radius,
                "calf_height":  calf_height,
                "calf_radius":  calf_radius,
                "foot_radius":  foot_radius
            }

        if mode == "symmetrical":
            # One random set for ALL legs
            dims_for_all = random_leg_dims()
            for leg_i in range(num_legs):
                leg_dims[leg_i] = dims_for_all

        elif mode == "bilateral":
            # Define which legs share the same dims (pairs) and which are solos
            # based on the special logic you gave:
            if num_legs == 2:
                pairs = [(0, 1)]
                solos = []
            elif num_legs == 3:
                pairs = [(0, 1)]
                solos = [2]
            elif num_legs == 4:
                pairs = [(0, 1), (2, 3)]
                solos = []
            elif num_legs == 5:
                pairs = [(0, 1), (2, 3)]
                solos = [4]
            elif num_legs == 6:
                pairs = [(0, 1), (2, 3), (4, 5)]
                solos = []
            else:
                # fallback: treat as all solos if you want,
                # or raise an error if not supported
                pairs = []
                solos = list(range(num_legs))

            # Assign dimensions for each pair or solo
            for (a, b) in pairs:
                dims_pair = random_leg_dims()
                leg_dims[a] = dims_pair
                leg_dims[b] = dims_pair

            for s in solos:
                leg_dims[s] = random_leg_dims()

        else:  # "random"
            # Fully randomized for every leg
            for leg_i in range(num_legs):
                leg_dims[leg_i] = random_leg_dims()

        return leg_dims

    def initialize_fixed_morphology_mass(self):  
        for i in range(len(self.links)): 
            self.links[i].set_params(mass=np.random.uniform(config.MASS_RANGE[0], config.MASS_RANGE[1]))

    def initialize_fixed_morphology_com(self):
        if config.MASS_DISTRIBUTION == "uniform":
            for i in range(len(self.links)):
                self.links[i].params['relative_com']=0.55
        elif config.MASS_DISTRIBUTION == "fixed":
            relative_com = np.random.uniform(config.COM_RANGE[0], config.COM_RANGE[1])
            for i in range(len(self.links)):
                self.links[i].set_params(relative_com=relative_com)
        elif config.MASS_DISTRIBUTION == "variable":
            for i in range(len(self.links)):
                self.links[i].set_params(relative_com=np.random.uniform(config.COM_RANGE[0], config.COM_RANGE[1]))
        else:
            raise ValueError("Invalid value for config.MASS_DISTRIBUTION")
        
    def initialize_fixed_morphology_size(self):
        """
        Assign sizes to the trunk, base, and each leg’s links based on conf.LEG_DIMENSION_MODE
        (hip, thigh, knee, calf, foot)
        """

        # Random trunk dimensions
        trunk_height = np.random.uniform(config.SIZE_RANGE_HEIGHT[0], config.SIZE_RANGE_HEIGHT[1])
        trunk_width  = np.random.uniform(config.SIZE_RANGE_WIDTH[0],  config.SIZE_RANGE_WIDTH[1])
        trunk_length = np.random.uniform(config.SIZE_RANGE_LENGTH[0], config.SIZE_RANGE_LENGTH[1])
        trunk_size   = [trunk_length, trunk_width, trunk_height]

        base_size = [0.01, 0.01, 0.01]

        #    For each leg, define random thigh/calf/foot geometry.
        #    We'll store them in a dict: {leg_index: {...}}
        dimension_mode = config.LEG_DIMENSION_MODE  # "random", "symmetrical", or "bilateral"

        leg_dims = self.assign_leg_dimensions(self.num_legs, dimension_mode)

        for link in self.links:
            name = link.link_name
            # If trunk or base, set trunk_size or base_size
            if isinstance(link, BoxLink):
                if name == 'base':
                    link.set_params(size=base_size)
                elif name == 'trunk':
                    link.set_params(size=trunk_size)

            # If it's "leg_X_thigh", "leg_X_calf", "leg_X_foot", "leg_X_hip", etc.
            elif name.startswith("leg_"):
                parts = name.split("_")  # e.g. ["leg", "0", "thigh"]
                if len(parts) < 3:
                    continue
                idx = int(parts[1])       # the leg index
                part = parts[2]          # "thigh", "calf", "foot", "hip", "knee"

                dims = leg_dims[idx]     # the dictionary of random dims for that leg

                if isinstance(link, SphereLink):
                    if part == "hip":
                        link.set_params(radius=dims["thigh_radius"])
                    elif part == "knee":
                        link.set_params(radius=max(dims["thigh_radius"], dims["calf_radius"]))
                    elif part == "foot":
                        link.set_params(radius=dims["foot_radius"])

                elif isinstance(link, CylinderLink):
                    if part == "thigh":
                        link.set_params(radius=dims["thigh_radius"],
                                        height=dims["thigh_height"])
                    elif part == "calf":
                        link.set_params(radius=dims["calf_radius"],
                                        height=dims["calf_height"])

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
            Ixx, Iyy, Izz = updated_inertia(link.params['relative_com'], inertia, position)
            link.params['inertia'] = [Ixx, 0, 0, Iyy, 0, Izz]
            link.params['com'] = position.com_pos
        
    def updated_params(self):
        self.updated_inertia_com()
        self.updated_origin()

        for i in range(len(self.links)):
            self.links[i].set_params(origin=self.links[i].params['origin'])

# Example usage
if __name__ == "__main__":
    robot = Robot(morphology='quadruped')

