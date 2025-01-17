import os
from robot import Robot
import config
from robot_builder.urdf_generator import URDFCreator
import shutil

   
class Population:
    def __init__(self, size, output_dir='../generations'):
        self.size = size
        self.robots = {}
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # If the output directory exists, delete all subdirectories inside it
        if os.path.exists(self.output_dir):
            for subdir in os.listdir(self.output_dir):
                subdir_path = os.path.join(self.output_dir, subdir)
                if os.path.isdir(subdir_path):
                    shutil.rmtree(subdir_path)  # Recursively delete subdirectory


    def initialize_population(self):
        # Create a population of robots
        for i in range(self.size):
            name = f"robot_{i+1}"
            # Creating a robot
            robot = Robot()
            robot.initialize_parametrized_morphology()
            self.robots[name] = robot

    def save_generation(self, generation):
        for robot_name, robot in self.robots.items():
            gen_dir = os.path.join(self.output_dir, f'generation_{generation}')
            os.makedirs(gen_dir, exist_ok=True)

            urdf = URDFCreator()
            text_file = open(os.path.
            join(gen_dir, f'{robot_name}.urdf'), 'w')

            content = "<robot name=\"quadruped_robot\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
            content += "\n"

            # Save the robot data in a URDF file
            links = robot.links
            joints = robot.joints

            for link in links:
                link_name = link.link_name
                collision = link.params.get('collision')
                origin = link.params.get('origin')
                inertia = link.params.get('inertia')
                mass = link.params.get('mass')
                com = link.params.get('com')
                class_name = type(link).__name__
                if class_name == 'BoxLink':
                    size = link.params.get('size')
                    content += urdf.make_box_link(link_name=link_name, size=size, collision=collision, origin=origin, inertia=inertia, mass=mass, com=com)
                elif class_name == 'SphereLink':
                    radius = link.params.get('radius')
                    content += urdf.make_sphere_link(link_name=link_name, radius=radius, collision=collision, origin=origin, inertia=inertia, mass=mass, com=com)
                elif class_name  == 'CylinderLink':
                    radius = link.params.get('radius')
                    height = link.params.get('height')
                    content += urdf.make_cylinder_link(link_name=link_name, link_height=height, radius=radius, collision=collision, origin=origin, inertia=inertia, mass=mass, com=com)

            for joint in joints:
                joint_name = joint.get_joint_name()
                joint_type = joint.get_joint_type()
                parent = joint.get_parent().link_name  # Ensure parent link name is used
                child = joint.get_child().link_name  # Ensure child link name is used
                joint_pos = joint.get_joint_pos()
                axis = joint.get_axis()
                content += urdf.make_joint(joint_name=joint_name, type=joint_type, parent=parent, child=child, joint_pos=joint_pos, axis=axis)

            content += " </robot> "
            text_file.write(content)
            text_file.close()


    def get_population_fitness(self):
        return [robot.get_fitness for robot in self.robots.values()]


# --------------------------------------------------------------------------- #

# Example usage:
if __name__ == "__main__":
    population = Population(config.POPULATION_SIZE, ['mass', 'size', 'com'], output_dir='generations')
    population.initialize_population()
    