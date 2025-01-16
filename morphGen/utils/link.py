class Link():
    def __init__(self, unique_id, link_type, link_name, **kwargs):
        self.unique_id = unique_id
        self.link_type = link_type
        self.link_name = link_name
        self.params = kwargs
        self.params.setdefault('relative_com', 0.5)

    def __repr__(self):
        return f"Link({self.unique_id}, {self.link_type}, {self.link_name}, {self.params})"

    def set_param(self, key, value):
        self.params[key] = value

    def get_param(self, key):
        return self.params.get(key, None)

    def set_params(self, **kwargs):
        for key, value in kwargs.items():
            self.params[key] = value

    def get_params(self):
        return self.params
    

class BoxLink(Link):
    def __init__(self, unique_id, link_name, **kwargs):
        super().__init__(unique_id, "BoxLink", link_name, **kwargs)

    def __repr__(self):
        return f"BoxLink({self.unique_id}, {self.link_name}, {self.params})"

    def set_size(self, size):
        self.params['size'] = size

    def get_size(self):
        return self.params.get('size', [0, 0, 0])

class CylinderLink(Link):
    def __init__(self, unique_id, link_name, **kwargs):
        super().__init__(unique_id, "CylinderLink", link_name, **kwargs)

    def __repr__(self):
        return f"CylinderLink({self.unique_id}, {self.link_name}, {self.params})"

    def set_dimensions(self, **kwargs):
        if 'radius' in kwargs:
            self.params['radius'] = kwargs['radius']
        if 'height' in kwargs:
            self.params['height'] = kwargs['height']

    def get_dimensions(self):
        return self.params.get('radius', 0), self.params.get('height', 0)

class SphereLink(Link):
    def __init__(self, unique_id, link_name, **kwargs):
        super().__init__(unique_id, "SphereLink", link_name, **kwargs)

    def __repr__(self):
        return f"SphereLink({self.unique_id}, {self.link_name}, {self.params})"

    def set_radius(self, radius):
        self.params['radius'] = radius

    def get_radius(self):
        return self.params.get('radius', 0)

   