from omni.isaac.lab.app import AppLauncher

# args.headless = True
app_launcher = AppLauncher()  # Adjust if any args required for your launcher
simulation_app = app_launcher.app

import os
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.app

from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
from omni.isaac.lab.utils.assets import check_file_path

from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
from omni.isaac.lab.utils.assets import check_file_path

import time
import sys
from tqdm import tqdm
from Evolutionary_algorithm import config


# The new URDF to USD conversion code
class HelloWorld:
    def __init__(self) -> None:
        pass

    def setup_scene(self, generation):
        # Define the relative path from the current working directory or script's location
        relative_path = f"generations/generation_{generation}"
        main_directory = os.path.join(os.getcwd(), relative_path)

        # Start timing for the total conversion process
        gen_start_time = time.time()

        # Collect all URDF files that need to be processed
        urdf_files = []
        for root, dirs, files in os.walk(main_directory):
            for file_name in files:
                if file_name.endswith(".urdf"):
                    urdf_files.append(os.path.join(root, file_name))

        # Process URDF files and track progress using tqdm
        with tqdm(total=len(urdf_files), desc="Converting URDF files to USD", dynamic_ncols=True) as pbar:
            for urdf_file_path in urdf_files:
                usda_file_name = os.path.basename(urdf_file_path).replace(".urdf", ".usda")
                usda_file_path = os.path.join(os.path.dirname(urdf_file_path), usda_file_name)

                # New URDF conversion logic
                if not os.path.isabs(urdf_file_path):
                    urdf_file_path = os.path.abspath(urdf_file_path)
                if not check_file_path(urdf_file_path):
                    raise ValueError(f"Invalid file path: {urdf_file_path}")
                
                dest_path = os.path.abspath(usda_file_path)

                # Create URDF converter config
                urdf_converter_cfg = UrdfConverterCfg(
                    asset_path=urdf_file_path,
                    usd_dir=os.path.dirname(dest_path),
                    usd_file_name=os.path.basename(dest_path),
                    fix_base=False,
                    merge_fixed_joints=False,
                    import_inertia_tensor=True,
                    self_collision=True,
                    force_usd_conversion=True,
                    make_instanceable=False,
                )

                # Create URDF converter and convert the file
                urdf_converter = UrdfConverter(urdf_converter_cfg)

                # Delete the URDF file after conversion if necessary
                os.remove(urdf_file_path)

                # Update progress bar after processing each file
                pbar.update(1)

        # End timing for the total conversion process
        gen_end_time = time.time()
        gen_elapsed_time = gen_end_time - gen_start_time
        print(f"Total time taken for conversions for generation {generation}: {gen_elapsed_time:.2f} seconds")

        if generation == config.NUM_GENERATIONS:
            simulation_app.close()


if __name__ == "__main__":
    # Start the simulation app using AppLauncher and new URDF conversion
    sample = HelloWorld()
    sample.setup_scene(generation=0)  # Pass a generation index here
    
    # Close the simulation app after processing
    simulation_app.close()
