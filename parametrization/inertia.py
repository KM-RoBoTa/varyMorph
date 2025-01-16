import xml.etree.ElementTree as ET
import math
import os
import numpy as np

class Inertia:
    def __init__(self, rho=0, r=0, h=0, x=0, y=0, z=0, type='CylinderLink', m=0):
        self.m = m
        self.r = r
        self.h = h
        self.x = x
        self.y = y
        self.z = z
        self.rho = rho
        self.type = type
        self.Ixx, self.Iyy, self.Izz = self.calculate_inertia()

    def calculate_mass(self):
        if self.type == 'BoxLink':
            return self.x * self.y * self.z * self.rho
        elif self.type == 'CylinderLink':
            return self.rho * self.h * self.r**2 * math.pi
        elif self.type == 'SphereLink':
            return self.rho * (4/3) * math.pi * self.r**3
        else:
            raise ValueError("Unsupported geometry type")

    def calculate_inertia(self):
        if self.m == 0:
            self.m = self.calculate_mass()

        if self.type == 'BoxLink':
            return self.inertia_box()
        elif self.type == 'CylinderLink':
            return self.inertia_cylinder()
        elif self.type == 'SphereLink':
            return self.inertia_sphere()
        else:
            raise ValueError("Unsupported geometry type")

    def inertia_box(self):
        I_xx = (1/12) * self.m * (self.y**2 + self.z**2)
        I_yy = (1/12) * self.m * (self.x**2 + self.z**2)
        I_zz = (1/12) * self.m * (self.x**2 + self.y**2)
        return I_xx, I_yy, I_zz

    def inertia_cylinder(self):
        I_xx = I_yy = (1/12) * self.m * (3 * self.r**2 + self.h**2)
        I_zz = 0.5 * self.m * self.r**2
        return I_xx, I_yy, I_zz

    def inertia_sphere(self):
        I_xx = I_yy = I_zz = (2/5) * self.m * self.r**2
        return I_xx, I_yy, I_zz

class Com_position:
    def __init__(self, r=0, h=0, x=0, y=0, z=0, rpy=None, type='cylinder', **kwargs):
        self.r = r
        self.h = h
        self.x = x
        self.y = y
        self.z = z
        self.rpy = rpy if rpy else [0, 0, 0]
        self.type = type
        self.relative_com = kwargs.get('relative_com', 0.5)
        self.com_pos = self.calculate_com_position()

    def calculate_com_position(self):
        if self.type == 'BoxLink':
            return self.com_position_box()
        elif self.type == 'CylinderLink':
            return self.com_position_cylinder()
        elif self.type == 'SphereLink':
            return self.com_position_sphere()
        else:
            raise ValueError("Unsupported geometry type")

    def com_position_box(self):
        x_pos = 0
        y_pos = 0
        z_pos = -self.z * self.relative_com
        return self.apply_rotation(x_pos, y_pos, z_pos, self.rpy)

    def com_position_cylinder(self):
        x_pos = 0
        y_pos = 0
        z_pos = -self.h * self.relative_com
        return self.apply_rotation(x_pos, y_pos, z_pos, self.rpy)

    def com_position_sphere(self):
        x_pos = 0
        y_pos = 0
        z_pos = self.r
        return self.apply_rotation(x_pos, y_pos, z_pos, self.rpy)

    def apply_rotation(self, x, y, z, rpy):
        roll, pitch, yaw = rpy

        Rr = self.rotation_matrix_x(roll)
        Rp = self.rotation_matrix_y(pitch)
        Ry = self.rotation_matrix_z(yaw)

        R = Ry @ Rp @ Rr

        pos = np.array([x, y, z])
        rotated_pos = R @ pos

        self.x = rotated_pos[0]
        self.y = rotated_pos[1]
        self.z = rotated_pos[2]

        return (rotated_pos[0], rotated_pos[1], rotated_pos[2])

    def rotation_matrix_x(self, angle):
        return np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])

    def rotation_matrix_y(self, angle):
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])

    def rotation_matrix_z(self, angle):
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0],
                         [0, 0, 1]])

def updated_inertia(relative_pos_com, inertia_obj, position_obj):
    original_rpy = position_obj.rpy
    new_rpy = [-angle for angle in original_rpy]

    if position_obj.type == 'CylinderLink':
        return update_inertia_cylinder(relative_pos_com, inertia_obj, position_obj, new_rpy, original_rpy)
    elif position_obj.type == 'BoxLink':
        return update_inertia_box(relative_pos_com, inertia_obj, position_obj, new_rpy, original_rpy)
    elif position_obj.type == 'SphereLink':
        return inertia_obj.calculate_inertia()
    else:
        raise ValueError("Unsupported geometry type")

def update_inertia_cylinder(relative_pos_com, inertia_obj, position_obj, new_rpy, original_rpy):
    position_obj.apply_rotation(position_obj.x, position_obj.y, position_obj.z, new_rpy)
    length_l = relative_pos_com * position_obj.h / 2
    factor_k = (inertia_obj.h / 2 - length_l) / length_l

    inertia1 = Inertia(r=inertia_obj.r, h=inertia_obj.h * (1 - relative_pos_com), type=inertia_obj.type, m=inertia_obj.m / (1 + factor_k))
    inertia2 = Inertia(r=inertia_obj.r, h=inertia_obj.h * relative_pos_com, type=inertia_obj.type, m=inertia_obj.m * factor_k / (1 + factor_k))

    Ixx_tot = Iyy_tot = inertia1.calculate_inertia()[0] + inertia2.calculate_inertia()[0] + inertia1.m * (inertia1.h / 2)**2 + inertia2.m * (inertia2.h / 2)**2
    Izz_tot = inertia1.calculate_inertia()[2] + inertia2.calculate_inertia()[2]

    position_obj.apply_rotation(position_obj.x, position_obj.y, position_obj.z, original_rpy)
    return Ixx_tot, Iyy_tot, Izz_tot

def update_inertia_box(relative_pos_com, inertia_obj, position_obj, new_rpy, original_rpy):
    position_obj.apply_rotation(position_obj.x, position_obj.y, position_obj.z, new_rpy)
    length_l = relative_pos_com * position_obj.z / 2
    factor_k = (inertia_obj.z / 2 - length_l) / length_l

    inertia1 = Inertia(x=inertia_obj.x, y=inertia_obj.y, z=inertia_obj.z * (1 - relative_pos_com), type=inertia_obj.type, m=inertia_obj.m / (1 + factor_k))
    inertia2 = Inertia(x=inertia_obj.x, y=inertia_obj.y, z=inertia_obj.z * relative_pos_com, type=inertia_obj.type, m=inertia_obj.m * factor_k / (1 + factor_k))

    Ixx_tot = Iyy_tot = inertia1.calculate_inertia()[0] + inertia2.calculate_inertia()[0] + inertia1.m * (inertia1.z / 2)**2 + inertia2.m * (inertia2.z / 2)**2
    Izz_tot = inertia1.calculate_inertia()[2] + inertia2.calculate_inertia()[2]

    position_obj.apply_rotation(position_obj.x, position_obj.y, position_obj.z, original_rpy)
    return Ixx_tot, Iyy_tot, Izz_tot

def update_urdf_with_inertia(urdf_file, link_name, inertia_obj):
    try:
        tree = ET.parse(urdf_file)
    except FileNotFoundError:
        print(f"File {urdf_file} not found. Please check the path.")
        return
    except ET.ParseError as e:
        print(f"Error parsing {urdf_file}: {e}")
        return

    root = tree.getroot()
    I_xx, I_yy, I_zz = inertia_obj.calculate_inertia()

    link_found = False
    for link in root.findall('link'):
        if link.get('name') == link_name:
            link_found = True
            visual = link.find('visual')
            if visual is not None:
                geometry = visual.find('geometry')
                if geometry is not None:
                    if inertia_obj.type == 'box' and geometry.find('box') is not None:
                        geom_match = True
                    elif inertia_obj.type == 'cylinder' and geometry.find('cylinder') is not None:
                        geom_match = True
                    elif inertia_obj.type == 'sphere' and geometry.find('sphere') is not None:
                        geom_match = True
                    else:
                        geom_match = False

                    if geom_match:
                        inertial = link.find('inertial')
                        if inertial is None:
                            inertial = ET.SubElement(link, 'inertial')
                            ET.SubElement(inertial, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
                            mass_tag = ET.SubElement(inertial, 'mass')
                            origin_tag = ET.SubElement(inertial, 'origin')
                            inertia_tag = ET.SubElement(inertial, 'inertia')
                        else:
                            mass_tag = inertial.find('mass')
                            origin_tag = inertial.find('origin')
                            inertia_tag = inertial.find('inertia')

                        mass_tag.set('value', str(inertia_obj.m))

                        origin_tag.set('xyz', f'{inertia_obj.x} {inertia_obj.y} {inertia_obj.z}')

                        inertia_tag.set('ixx', str(I_xx))
                        inertia_tag.set('ixy', '0')
                        inertia_tag.set('ixz', '0')
                        inertia_tag.set('iyy', str(I_yy))
                        inertia_tag.set('iyz', '0')
                        inertia_tag.set('izz', str(I_zz))
                    else:
                        print(f"Geometry does not match for link {link_name}. Expected {inertia_obj.type}.")
                else:
                    print(f"No geometry found for link {link_name}.")
            else:
                print(f"No visual element found for link {link_name}.")

    if link_found and geom_match:
        tree.write(urdf_file)
        print(f'URDF file {urdf_file} updated successfully.')
    else:
        print(f'Link {link_name} not found in {urdf_file}.')

def update_urdf_with_position(urdf_file, link_name, position_obj):
    try:
        tree = ET.parse(urdf_file)
    except FileNotFoundError:
        print(f"File {urdf_file} not found. Please check the path.")
        return
    except ET.ParseError as e:
        print(f"Error parsing {urdf_file}: {e}")
        return

    root = tree.getroot()
    (x_pos, y_pos, z_pos) = position_obj.calculate_com_position()

    link_found = False
    geom_match = False

    for link in root.findall('link'):
        if link.get('name') == link_name:
            link_found = True
            visual = link.find('visual')
            if visual is not None:
                geometry = visual.find('geometry')
                if geometry is not None:
                    if position_obj.type == 'box':
                        geom_element = geometry.find('box')
                    elif position_obj.type == 'cylinder':
                        geom_element = geometry.find('cylinder')
                    elif position_obj.type == 'sphere':
                        geom_element = geometry.find('sphere')
                    else:
                        geom_element = None

                    if geom_element is not None:
                        geom_match = True
                    else:
                        geom_match = False

                    if geom_match:
                        origin_tag = visual.find('origin')
                        if origin_tag is None:
                            origin_tag = ET.SubElement(visual, 'origin')

                        origin_tag.set('xyz', f'{x_pos} {y_pos} {z_pos}')

                        # Update geometry based on the type
                        if position_obj.type == 'box':
                            if geom_element is None:
                                geom_element = ET.SubElement(geometry, 'box')
                            geom_element.set('size', f'{position_obj.x} {position_obj.y} {position_obj.z}')
                        elif position_obj.type == 'cylinder':
                            if geom_element is None:
                                geom_element = ET.SubElement(geometry, 'cylinder')
                            geom_element.set('radius', str(position_obj.r))
                            geom_element.set('length', str(position_obj.h))
                        elif position_obj.type == 'sphere':
                            if geom_element is None:
                                geom_element = ET.SubElement(geometry, 'sphere')
                            geom_element.set('radius', str(position_obj.r))
                    else:
                        print(f"Geometry does not match for link {link_name}. Expected {position_obj.type}.")
                else:
                    print(f"No geometry found for link {link_name}.")
            else:
                print(f"No visual element found for link {link_name}.")

    if link_found and geom_match:
        tree.write(urdf_file)
        print(f'URDF file {urdf_file} updated successfully.')
    else:
        if not link_found:
            print(f'Link {link_name} not found in {urdf_file}.')
        if not geom_match:
            print(f'Geometry type {position_obj.type} does not match for link {link_name}.')

