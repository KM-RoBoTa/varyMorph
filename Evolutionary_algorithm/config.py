# --------------------------------------------------------------------------- #
# Evolution Options
# --------------------------------------------------------------------------- #


# Population size
POPULATION_SIZE = 50

# Number of generations to evolve
NUM_GENERATIONS = 2

# Resizing factor
RES_FACTOR = 1

SIZE_RANGE_LENGTH = [0.8*RES_FACTOR, 1.2*RES_FACTOR]
SIZE_RANGE_WIDTH = [0.4*RES_FACTOR, 0.6*RES_FACTOR]
SIZE_RANGE_HEIGHT = [0.15*RES_FACTOR, 0.3*RES_FACTOR]
HEIGHT_RANGE = [0.3*RES_FACTOR, 0.6*RES_FACTOR]
MASS_RANGE = [0.5*(RES_FACTOR)**3, 2*(RES_FACTOR)**3]
RADIUS_RANGE = [0.08*RES_FACTOR, 0.11*RES_FACTOR]
JOINT_POS_RANGE = [[-0.02, -0.02, -0.02], [0.02, 0.02, 0.02]]
FOOT_SPHERE_RADIUS = 0.2*RES_FACTOR
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

