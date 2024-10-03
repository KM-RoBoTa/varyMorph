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
     cd ~/GitHub/Intelligent-robot-design/Evolutionary_algorithm
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
