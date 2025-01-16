## varyMorph, a Tool to Automatically Generate Legged Robot Morphologies.
Designing efficient and robust legged robots for navigating diverse terrains is a central challenge in robotics research. To simplify this process, we introduce varyMorph, an open-source tool capable of programmatically generating a wide variety of legged robot models in both URDF and USDA formats. By randomizing attributes such as the number of legs, morphology, body dimensions, and mass distribution, varyMorph can produce hundreds or even thousands of valid robot designs within minutes. These models are easily compatible with popular simulators like Isaac Sim/Lab, MuJoCo, Isaac Gym, and RaiSim, enabling large-scale experimentation in areas such as evolutionary robotics and reinforcement learning. Our findings demonstrate near-linear scalability in terms of model complexity and batch size, with fast generation achievable on both high-performance and standard desktop hardware. This high-throughput approach to morphological exploration supports data-driven optimization and opens avenues for innovative research in locomotion control, sim-to-real transfer, and co-evolutionary algorithms. We position varyMorph as a complementary tool to grammar-based, latent-space, and gradient-driven design methodologies, lowering the barriers to discovering novel and resilient robot morphologies.

## Walkthrough: How to Execute the Conversion Process

1. **Create the Conda Environment**
   - Run the following command to create the environment with Python 3.10 and install the required dependencies from `requirements.txt`:
   ```bash
   conda create --name varymorph python=3.10 --file requirements.txt
   ```
   - **For URDF files only**: No additional installation is required.
   - **For USDA files (IsaacSim/Lab compatibility)**: Ensure you have **Isaac Sim 4.0.0+** and **Isaac Lab 1.1.0+** installed.

2. **Activate the Conda Environment:**
   - Open a terminal.
   - Activate the required conda environment by running:
     ```bash
     conda activate varymorph
     ```

3. **Navigate to the Project Directory:**
   - Change your directory to the project folder:
     ```bash
     cd varyMorph/morphGen
     ```

4. **Execute the Morphology Generation Process:**
   - Run the evolutionary algorithm as before by executing:
     ```bash
     python varyMorph.py
     ```
   - This will automatically generate USDA/URDF files depending on config file setup.\


## Explaining the config file
- The directory of the config file exists at the following path:
     ```bash
     cd morphGen
     ```
The `config.py` file defines key parameters for generating robotic morphologies.

- `FILE_TYPE`: Specifies the output format, either `"URDF"` or `"USD"`.
- `POPULATION_SIZE`: Defines the number of robot morphologies to generate.
- `MORPHOLOGY`: Determines the type of legged robot (e.g., biped, tripod, quadruped).
- `LEG_DIMENSION_MODE`: Controls how leg dimensions are assigned (`random`, `symmetrical`, or `bilateral`).
- `RES_FACTOR`: A scaling factor applied to all size-related parameters.

### Size Ranges for the Base:
- `SIZE_RANGE_LENGTH`, `SIZE_RANGE_WIDTH`, `SIZE_RANGE_HEIGHT`: Define the allowable dimensions of the robot’s body, scaled by `RES_FACTOR`.

### Other Ranges for the Legs:
- `HEIGHT_RANGE`: Sets the possible height variations.
- `MASS_RANGE`: Determines the mass values, scaled proportionally to `RES_FACTOR³`.
- `RADIUS_RANGE`: Specifies the size range of cylindrical or spherical components.

### Mass Distribution and Center of Mass (COM):
-  `Center of mass (COM)`: relative positioning within a link
- `MASS_DISTRIBUTION`: Defines how the center of mass is assigned:
  - `"uniform"`: COM is in the middle of each link (uniform).
  - `"fixed"`: A single random COM value is applied to all links.
  - `"variable"`: Each link gets a different COM value.
- `COM_RANGE`: Specifies the possible relative positioning of the COM within a link.

This configuration file ensures that all generated robotic morphologies follow a structured and controlled approach to size, mass, and mechanical properties.
