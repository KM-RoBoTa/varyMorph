import os
import sys

sys.path.append(os.path.abspath('..'))
from Evolutionary_algorithm.new_link import *
from Evolutionary_algorithm.urdf_generator import *
from Parametrization.inertia import *


def create_box_link(id, shape_type, link_name, **kwargs):

    size = kwargs.get("size", [0.01, 0.01, 0.01])
    mass = kwargs.get("mass", 0)
    rho = kwargs.get("rho", 0)                              
    relative_com_pos = kwargs.get("relative_com", 0.5)      # SHOULD USE THE CONFIG.MASS_DISTRIBUTION TO DEFINE THE DEFAUL VALUE

    if mass == 0 and rho == 0:
        raise ValueError("Either mass or rho should be provided")
    if mass != 0 and rho != 0:
        raise ValueError("Both mass and rho cannot be provided")
    
    box_link_inertia = Inertia(x=size[0], y=size[1], z=size[2], rho=rho, m= mass, type=shape_type)
    Ixx, Iyy, Izz = box_link_inertia.calculate_inertia()
    box_link_position = Com_position(x=size[0], y=size[1], z=size[2], rpy=[0, 0, 0], type=shape_type, relative_com=relative_com_pos)

    if relative_com_pos != 0.5:
        Ixx, Iyy, Izz = updated_inertia_com(relative_com_pos, box_link_inertia, box_link_position)
    
    box_link = BoxLink(unique_id=f"{link_name}_{id}", link_name=link_name, size=size, collision=True, origin=(0, 0, 0), inertia=[Ixx, 0, 0, Iyy, 0, Izz], mass=box_link_inertia.m, com=box_link_position.com_pos)

    # if relative_com_pos != 0.5:
    #     print(f"The parametrization is for center of mass")
    # elif mass == 0:
    #     print(f"The parametrization is for volume")
    # elif rho == 0:
    #     print(f"The parametrization is for mass")
    
    return box_link

def create_cylinder_link(id, shape_type, link_name, **kwargs):

    radius = kwargs.get("radius", 0.025)
    height = kwargs.get("height", 0.05)
    mass = kwargs.get("mass", 0)
    rho = kwargs.get("rho", 0)
    relative_com_pos = kwargs.get("relative_com", 0.5)

    if mass == 0 and rho == 0:
        raise ValueError("Either mass or rho should be provided")
    if mass != 0 and rho != 0:
        raise ValueError("Both mass and rho cannot be provided")
    
    cylinder_link_inertia = Inertia(r=radius, h=height, rho=rho, m=mass, type=shape_type)
    Ixx, Iyy, Izz = cylinder_link_inertia.calculate_inertia()
    cylinder_link_position = Com_position(r=radius, h=height, type=shape_type, relative_com=relative_com_pos)

    if relative_com_pos != 0.5:
        Ixx, Iyy, Izz = updated_inertia_com(relative_com_pos, cylinder_link_inertia, cylinder_link_position)
    
    cylinder_link = CylinderLink(unique_id=f"{link_name}_{id}", link_name=link_name, height=height, radius=radius, collision=True, origin=(0, 0, 0), inertia=[Ixx, 0, 0, Iyy, 0, Izz], mass=cylinder_link_inertia.m, com=cylinder_link_position.com_pos)

    # if relative_com_pos != 0.5:
    #     print(f"The parametrization is for center of mass")
    # elif mass == 0:
    #     print(f"The parametrization is for volume")
    # elif rho == 0:
    #     print(f"The parametrization is for mass")

    return cylinder_link

def create_sphere_link(id, shape_type, link_name, **kwargs):

    radius = kwargs.get("radius", 0.025)
    mass = kwargs.get("mass", 0)
    rho = kwargs.get("rho", 0)
    relative_com_pos = kwargs.get("relative_com", 0.5)

    if mass == 0 and rho == 0:
        raise ValueError("Either mass or rho should be provided")
    if mass != 0 and rho != 0:
        raise ValueError("Both mass and rho cannot be provided")
    
    sphere_link_inertia = Inertia(r=radius, rho=rho, m=mass, type=shape_type)
    Ixx, Iyy, Izz = sphere_link_inertia.calculate_inertia()
    sphere_link_position = Com_position(r=radius, type=shape_type)

    sphere_link = SphereLink(unique_id=f"{link_name}_{id}", link_name=link_name, radius=radius, collision=True, origin=(0, 0, 0), inertia=[Ixx, 0, 0, Iyy, 0, Izz], mass=sphere_link_inertia.m, com=sphere_link_position.com_pos)
    
    # if relative_com_pos != 0.5:
    #     raise ValueError("Parametrization of center of mass is not applicable for sphere")
    # elif mass == 0:
    #     print(f"The parametrization is for volume")
    # elif rho == 0:
    #     print(f"The parametrization is for mass")
    
    return sphere_link

def create_link(id, shape_type, link_name, **kwargs):
    if shape_type == "BoxLink":
        return create_box_link(id, shape_type, link_name, **kwargs)
    elif shape_type == "CylinderLink":
        return create_cylinder_link(id, shape_type, link_name, **kwargs)
    elif shape_type == "SphereLink":
        return create_sphere_link(id, shape_type, link_name, **kwargs)
    else:
        raise ValueError("Unsupported geometry type")
    
def parametrization(id, shape_type, link_name, parameter, **kwargs):
    relative_com = kwargs.get("relative_com", 0.5)
    mass = kwargs.get("mass", 0)
    rho = kwargs.get("rho", 0)
    x = kwargs.get("x", 0.01)
    y = kwargs.get("y", 0.01)
    z = kwargs.get("z", 0.01)
    radius = kwargs.get("radius", 0.01)
    height = kwargs.get("height", 0.01)

    link = None

    if parameter == "center of mass" and relative_com != 0.5:
        link = create_link(id=id, shape_type=shape_type, link_name=link_name, relative_com=relative_com, mass=mass, rho=rho, x=x, y=y, z=z, radius=radius, height=height)
    elif parameter == "center of mass" and relative_com == 0.5:
        raise ValueError("Center of mass parameter should be provided")    
    elif parameter == "volume" and mass == 0:
        link = create_link(id=id, shape_type=shape_type, link_name=link_name, relative_com=relative_com, mass=mass, rho=rho, x=x, y=y, z=z, radius=radius, height=height)
    elif parameter == "volume" and mass != 0:
        raise ValueError("Volume parameter should be provided")
    elif parameter == "mass" and rho == 0:
        link = create_link(id=id, shape_type=shape_type, link_name=link_name, relative_com=relative_com, mass=mass, rho=rho, x=x, y=y, z=z, radius=radius, height=height)
    elif parameter == "mass" and rho !=0:
        raise ValueError("Mass parameter should be provided")
    else:
        raise ValueError("Invalid parameter")

    return link
