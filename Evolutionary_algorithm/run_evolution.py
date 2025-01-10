from population import *
from evolution_methods import *
import config
import os
import sys
import json




if __name__ == "__main__":
    population = Population(config.POPULATION_SIZE, ['mass', 'size', 'com'], output_dir='generations')
    population.initialize_population()
    # print the fitness of the population
    print("initial fitness of the population", np.mean(population.get_population_fitness()))
    print("initial fitness of the population", population.get_population_fitness())
    evolution = Evolution(population)
    population.save_generation(0)

    if config.FILE_TYPE == "USD":
        from usd_generator.urdf_importer import HelloWorld
        sample = HelloWorld()
        sample.setup_scene(0)

    # for generation in range(config.NUM_GENERATIONS):
    #     evolution.evolve()
    #     population.save_generation(generation + 1)
    #     sample.setup_scene(generation+1)

    print("final fitness: ", np.mean(population.get_population_fitness())) 




