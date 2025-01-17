# --------------------------------------------------------------------------- #
# Morphology Options
# --------------------------------------------------------------------------- #

FILE_TYPE = "URDF"  # "URDF" or "USDA"

# Population size
POPULATION_SIZE = 50

MORPHOLOGY = "tripod"    # "biped", "tripod", "quadruped", "pentapod", or "hexapod"

# Mode for dimensions of legs
LEG_DIMENSION_MODE = "symmetrical"  # "random" or "symmetrical" or "bilateral"

# Resizing factor
RES_FACTOR = 1

# Size Ranges for the Base
SIZE_RANGE_LENGTH = RES_FACTOR * [0.8, 1.5]
SIZE_RANGE_WIDTH = RES_FACTOR * [0.1, 0.8]
SIZE_RANGE_HEIGHT = RES_FACTOR * [0.15, 0.3]

# Size ranges for the Legs
HEIGHT_RANGE = RES_FACTOR * [0.25, 0.9]
MASS_RANGE = [(0.5 * RES_FACTOR ** 3), (2 * RES_FACTOR ** 3)]  # Scaled by RES_FACTOR³
RADIUS_RANGE = RES_FACTOR * [0.08, 0.11]


# Range for the center of mass (COM) relative positioning within a link.
# - "uniform": center of mass is in middle of link.
# - "variable": A different COM value is assigned to each link.
# - "fixed": A single random COM value is applied to all links.
MASS_DISTRIBUTION = "fixed" # "uniform", "fixed", or "variable"
COM_RANGE = [0.1, 0.9]