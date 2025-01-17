from population import *
import config as config


if __name__ == "__main__":
    population = Population(config.POPULATION_SIZE, ['mass', 'size', 'com'])
    population.initialize_population()
    population.save_generation(0)

    if config.FILE_TYPE == "USDA":
        from robot_builder.usd_generator import URDFtoUSD
        sample = URDFtoUSD()
        sample.setup_conversion(0)

    

    print("Morphology generation completed successfully!") 




