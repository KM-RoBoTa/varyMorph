from link import Link
from joint import Joint
from Evolutionary_algorithm.robot import Robot

def create_initial_population(size):
    robots = []
    for i in range(size):
        links = [
            Link(unique_id=f"base_{i}", link_type="box", link_name="base", size=[0.01, 0.01, 0.01], collision=False, origin=[0.0, 0.0, 0.0], inertia=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], mass=0.0, com=[0.0, 0.0, 0.0]),
            # Add other links here
        ]
        joints = [
            Joint(unique_id=f"floating_base_{i}", joint_name="floating_base", joint_type="fixed", parent="base", child="trunk", joint_pos=[0.0, 0.0, 0.0], axis=[0, 0, 0]),
            # Add other joints here
        ]
        robot = Robot(links, joints)
        robots.append(robot)
    return robots

if __name__ == "__main__":
    initial_population = create_initial_population(size=10)
    evolution = Evolution(population_size=10, initial_robots=initial_population)
    evolution.run(generations=50)
