import os
import sys
sys.path.append(os.path.abspath('..'))


class Joint:
    def __init__(self, joint_name, joint_type, parent, child, joint_pos=None, axis=None):
        self.joint_name = joint_name
        self.joint_type = joint_type
        self.parent = parent
        self.child = child
        self.joint_pos = joint_pos if joint_pos is not None else [0, 0, 0]
        self.axis = axis if axis is not None else [0, 0, 0]

    def __repr__(self):
        return (f"Joint({self.joint_name}, {self.joint_type}, "
                f"parent={self.parent}, child={self.child}, "
                f"joint_pos={self.joint_pos}, axis={self.axis})")

    def set_joint_name(self, joint_name):
        self.joint_name = joint_name

    def get_joint_name(self):
        return self.joint_name

    def set_joint_type(self, joint_type):
        self.joint_type = joint_type

    def get_joint_type(self):
        return self.joint_type

    def set_parent(self, parent):
        self.parent = parent

    def get_parent(self):
        return self.parent

    def set_child(self, child):
        self.child = child

    def get_child(self):
        return self.child

    def set_joint_pos(self, joint_pos):
        self.joint_pos = joint_pos

    def get_joint_pos(self):
        return self.joint_pos

    def set_axis(self, axis):
        self.axis = axis

    def get_axis(self):
        return self.axis

    def set_params(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def get_params(self):
        return {
            "joint_name": self.joint_name,
            "joint_type": self.joint_type,
            "parent": self.parent,
            "child": self.child,
            "joint_pos": self.joint_pos,
            "axis": self.axis
        }
        
    def calculate_joint_pos(self):

        # Compare classes directly with correct module paths
        parent_class_name = type(self.parent).__name__
        parent_module_name = type(self.parent).__module__

        # Updated checks with correct module path
        if parent_class_name == 'BoxLink' and parent_module_name == 'Evolutionary_algorithm.new_link':
            l, w, h = self.parent.params['size']
            return [0, 0, h / 2]
        elif parent_class_name == 'SphereLink' and parent_module_name == 'Evolutionary_algorithm.new_link':
            r = self.parent.params['radius']
            return [0, 0, r]
        elif parent_class_name == 'CylinderLink' and parent_module_name == 'Evolutionary_algorithm.new_link':
            h = self.parent.params['height']
            return [0, 0, h / 2]
        else:
            return [0, 0, 0]


