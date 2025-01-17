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
from morphGen import config


# The new URDF to USD conversion code
class URDFtoUSD:
    def __init__(self) -> None:
        pass

    def setup_conversion(self, generation):
        # Define the relative path from the current working directory or script's location
        relative_path = f"generations/generation_{generation}"
        main_directory = os.path.join(os.getcwd(), relative_path)

        # Collect all URDF files that need to be processed
        urdf_files = []
        for root, dirs, files in os.walk(main_directory):
            for file_name in files:
                if file_name.endswith(".urdf"):
                    urdf_files.append(os.path.join(root, file_name))

        num_robots_processed = []
        elapsed_times = []
        gen_start_time = time.time()

        with tqdm(total=len(urdf_files), desc="Converting URDF files to USDA", dynamic_ncols=True) as pbar:
            for i, urdf_file_path in enumerate(urdf_files, start=1):
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
                    merge_fixed_joints=True,
                    import_inertia_tensor=False,
                    self_collision=True,
                    force_usd_conversion=True,
                    make_instanceable=False,
                )

                # Create URDF converter and convert the file
                urdf_converter = UrdfConverter(urdf_converter_cfg)

                # Optionally remove the original URDF file
                os.remove(urdf_file_path)

                # Update progress bar after processing each file
                pbar.update(1)

        # End timing for the total conversion process
        gen_end_time = time.time()
        gen_elapsed_time = gen_end_time - gen_start_time
        print(f"Total time taken for conversions six legged for generation {generation}: {gen_elapsed_time:.2f} seconds")

        simulation_app.close()
