class BodyPart:
    def __init__(self, name, mass, position, inertia, com, size=None, radius=None, link_height=None, collision=False, part_type=None):
        self.name = name
        self.mass = mass
        self.position = position
        self.inertia = inertia
        self.com = com
        self.size = size
        self.radius = radius
        self.link_height = link_height
        self.collision = collision
        self.part_type = part_type
        

    def to_dict(self):
        return {
            "name": self.name,
            "mass": self.mass,
            "position": self.position,
            "orientation": self.orientation,
            "part_type": self.part_type,
            "size": self.size,
            "radius": self.radius,
            "link_height": self.link_height,
            "collision": self.collision,
            "inertia": self.inertia,
            "com": self.com
        }
