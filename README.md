## varyMorph, a Tool to Automatically Generate Legged Robot Morphologies.
Designing efficient and robust legged robots for navigating diverse terrains is a central challenge in robotics research. To simplify this process, we introduce varyMorph, an open-source tool capable of programmatically generating a wide variety of legged robot models in both URDF and USDA formats. By randomizing attributes such as the number of legs, morphology, body dimensions, and mass distribution, varyMorph can produce hundreds or even thousands of valid robot designs within minutes. These models are easily compatible with popular simulators like Isaac Sim/Lab, MuJoCo, Isaac Gym, and RaiSim, enabling large-scale experimentation in areas such as evolutionary robotics and reinforcement learning. Our findings demonstrate near-linear scalability in terms of model complexity and batch size, with fast generation achievable on both high-performance and standard desktop hardware. This high-throughput approach to morphological exploration supports data-driven optimization and opens avenues for innovative research in locomotion control, sim-to-real transfer, and co-evolutionary algorithms. We position varyMorph as a complementary tool to grammar-based, latent-space, and gradient-driven design methodologies, lowering the barriers to discovering novel and resilient robot morphologies.

## Walkthrough: How to Execute the Conversion Process

1. **Activate the Conda Environment:**
   - Open a terminal.
   - Activate the required conda environment by running:
     ```bash
     conda activate isaaclab
     ```

2. **Navigate to the Project Directory:**
   - Change your directory to the project folder:
     ```bash
     cd Intelligent-robot-design/Evolutionary_algorithm
     ```

3. **Execute the Evolutionary Algorithm:**
   - Run the evolutionary algorithm as before by executing:
     ```bash
     python run_evolution.py
     ```
   - This will automatically generate USDA files instead of URDF files.
   - It will also show how much time it took for conversion of all files.
   - The coversion rate is approximately 1000files/min.

## Explanation: What Was Done

1. **Script Creation:**
   - A new Python script was created to find URDF files in directories, convert them to USDA format using the Isaac Sim API, and delete the URDF files after successful conversion.
   - This script is located in the `usd_generator` folder.

2. **Automated Conversion:**
   - The script has been integrated into the existing workflow, ensuring that the conversion to USDA format happens automatically during the execution of the `run_evolution.py` script.

3. **Environment Setup:**
   - To execute the script, make sure to activate the conda environment named `isaaclab`. This environment is configured with all necessary dependencies, including Isaac Sim.

By following these steps, the evolutionary algorithm will now output USDA files directly, which optimizes the simulation process on our monster machine.
