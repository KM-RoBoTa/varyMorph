import numpy as np
import random
from new_link import BoxLink, SphereLink, CylinderLink
import config
from robot import Robot
from population import Population

class Evolution:
    def __init__(self, population, mutation_rate=config.MUTATION_RATE, elite_fraction=config.ELITE_FRACTION):
        self.population = population
        self.mutation = Mutation(mutation_rate)
        self.selection = TournamentSelection(population)
        self.elite_fraction = elite_fraction


    def evolve(self):
        selected_robots = self.selection.select()
        
        elite_count = int(len(self.population.robots) * self.elite_fraction)
        
        elites = self.get_elites(elite_count)
        
        # Generating the new generation without elites
        new_generation = {}
        for idx, robot in enumerate(selected_robots):
            new_robot = robot.copy()
            new_robot.fitness = None  # Reset fitness for recalculation
            self.mutation.select_mutate(new_robot)
            new_robot.updated_params()
            new_generation[f"robot_{idx + 1}"] = new_robot
        
        # Adding the elites back to the new generation
        for elite in elites:
            new_generation[f"robot_{len(new_generation) + 1}"] = elite
        
        # Sorting new generation by fitness and retain the best up to the original population size
        sorted_generation = sorted(new_generation.values(), key=lambda robot: robot.get_fitness, reverse=True)
        self.population.robots = {f"robot_{idx + 1}": sorted_generation[idx] for idx in range(len(self.population.robots))}
        print("fitness: ", self.population.get_population_fitness())


    def get_elites(self, elite_count):
        robots_sorted_by_fitness = sorted(self.population.robots.values(), key=lambda robot: robot.get_fitness, reverse=True)
        return robots_sorted_by_fitness[:elite_count]


class Mutation:
    def __init__(self, mutation_rate):
        self.mutation_rate = mutation_rate

    def mutate(self, robot):
        self.select_mutate(robot)

    def select_mutate(self, robot):
        mutation_methods = [self.mutate_mass]
        for link in robot.links:
            if random.random() < self.mutation_rate:
                method = random.choice(mutation_methods)
                method(link)

    def mutate_mass(self, link):
        if random.random() < self.mutation_rate:
            link.params['mass'] = np.random.uniform(config.MASS_RANGE[0], config.MASS_RANGE[1])

    def mutate_size(self, link):
        if isinstance(link, BoxLink) and random.random() < self.mutation_rate:
            link.params['size'] = [np.random.uniform(config.SIZE_RANGE[0], config.SIZE_RANGE[1]) for _ in range(3)]
        elif isinstance(link, SphereLink) and random.random() < self.mutation_rate:
            link.params['radius'] = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
        elif isinstance(link, CylinderLink) and random.random() < self.mutation_rate:
            link.params['radius'] = np.random.uniform(config.RADIUS_RANGE[0], config.RADIUS_RANGE[1])
            link.params['height'] = np.random.uniform(config.HEIGHT_RANGE[0], config.HEIGHT_RANGE[1])

        
    def mutate_joint(self, robot):
        for joint in robot.joints:
            if random.random() < self.mutation_rate:
                joint.axis = np.random.uniform(-1, 1, size=3).tolist()
                joint.limit = {
                    "effort": np.random.uniform(50, 150),
                    "velocity": np.random.uniform(30, 70)
                }
         
class TournamentSelection:
    def __init__(self, population, k=config.TOURNAMENT_SIZE):
        self.population = population
        self.k = k

    def select(self):
        winners = []
        population_size = len(self.population.robots)
        k = min(self.k, population_size)  # Ensure k is not larger than the population size
        for _ in range(len(self.population.robots)):
            contenders = random.sample(list(self.population.robots.values()), k)
            winner = max(contenders, key=lambda robot: robot.get_fitness)
            winners.append(winner)
        return winners

