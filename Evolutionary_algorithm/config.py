# --------------------------------------------------------------------------- #
# Evolution Options
# --------------------------------------------------------------------------- #

FILE_TYPE = "USD"

# Population size
POPULATION_SIZE = 10

# Mode for dimensions of legs
LEG_DIMENSION_MODE = "random"  # or "symmetrical" or "bilateral"

# Resizing factor
RES_FACTOR = 1

# Size Ranges
SIZE_RANGE_LENGTH = RES_FACTOR * [0.8, 1.5]
SIZE_RANGE_WIDTH = RES_FACTOR * [0.4, 0.8]
SIZE_RANGE_HEIGHT = RES_FACTOR * [0.15, 0.3]

# Other Ranges
HEIGHT_RANGE = RES_FACTOR * [0.25, 0.9]
MASS_RANGE = [(0.5 * RES_FACTOR ** 3), (2 * RES_FACTOR ** 3)]  # Scaled by RES_FACTOR³
RADIUS_RANGE = RES_FACTOR * [0.08, 0.11]

ELITE_FRACTION = 0.2

# type of morphology: quadruped, arbitrary
MORPHOLOGY = "quadruped"

MASS_DISTRIBUTION = "uniform"
DECISION_VARIABLES = ["mass", "size", "com"]

# Tournament type. 
TOURNAMENT_TYPE = "aging_num"
TOURNAMENT_SIZE = 2

# See EVO.TOURNAMENT_TYPE
AGING_WINDOW_SIZE = 100

# Number of participants in the tournament. 
NUM_PARTICIPANTS = 2

# Percent of the active population to use during tournament selection.
# Alternative to EVO.NUM_PARTICIPANTS. In case of vanila tournament active
# population will be the percent of unimals currenlty alive (i.e number of
# metadata files). For aging tournament it will be a percentage of
# AGING_WINDOW_SIZE.
PERCENT_PARTICIPANTS = 5

MUTATION_OPTIONS = [
"com"
]

MUTATION_RATE = 2

# a parameter showing which parameters to use for optimization


ELECTION_CRITERIA = ["total_mass"]

# Specify the objective corresponding to the criteria above. e.g you might want
# [-1, -1] to minimize mass and total mass. [1, 1] to maximize mass and total
SELECTION_CRITERIA_OBJ = [1]

