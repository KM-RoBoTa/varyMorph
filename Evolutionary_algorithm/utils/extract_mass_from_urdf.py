import xml.etree.ElementTree as ET
import numpy as np

def extract_mass(urdf_path):

    """It reads the URDF file and checks if the mass is defined for each link.
    
    input: urdf_path: path to the URDF file
    output: mass_list: list of tuples containing the link name, mass, position, and orientation
    
    """

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    mass_list = []

    for link in root.findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', 'Not specified')
                # make a tuple of x, y, z coordinates
                xyz = tuple(map(float, xyz.split()))
                rpy = origin.get('rpy', 'Not specified')
                # make a tuple of roll, pitch, yaw
                rpy = tuple(map(float, rpy.split()))
                mass = float(inertial.find('mass').get('value'))
                # add the mass value, name, and its position to the list, each element is a tuple
                mass_list.append((name, mass, xyz, rpy))
                #print(f"Link: {name}, Mass: {mass} Mass Position: {xyz}, Orientation: {rpy}")

    
    return mass_list    


mass_list = extract_mass('robot_sample.urdf')
print(mass_list)

