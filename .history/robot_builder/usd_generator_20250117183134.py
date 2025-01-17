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


import plotly.express as px
import matplotlib as plt
import pandas as pd


# The new URDF to USD conversion code
class URDFtoUSD:
    def __init__(self) -> None:
        pass

    def setup_conversion(self, generation):
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

        # # Process URDF files and track progress using tqdm
        # with tqdm(total=len(urdf_files), desc="Converting URDF files to USD", dynamic_ncols=True) as pbar:
        #     for urdf_file_path in urdf_files:
        #         usda_file_name = os.path.basename(urdf_file_path).replace(".urdf", ".usda")
        #         usda_file_path = os.path.join(os.path.dirname(urdf_file_path), usda_file_name)

        #         # New URDF conversion logic
        #         if not os.path.isabs(urdf_file_path):
        #             urdf_file_path = os.path.abspath(urdf_file_path)
        #         if not check_file_path(urdf_file_path):
        #             raise ValueError(f"Invalid file path: {urdf_file_path}")
                
        #         dest_path = os.path.abspath(usda_file_path)

        #         # Create URDF converter config
        #         urdf_converter_cfg = UrdfConverterCfg(
        #             asset_path=urdf_file_path,
        #             usd_dir=os.path.dirname(dest_path),
        #             usd_file_name=os.path.basename(dest_path),
        #             fix_base=False,
        #             merge_fixed_joints=True,
        #             import_inertia_tensor=False,
        #             self_collision=True,
        #             force_usd_conversion=True,
        #             make_instanceable=False,
        #         )

        #         # Create URDF converter and convert the file
        #         urdf_converter = UrdfConverter(urdf_converter_cfg)

        #         # Delete the URDF file after conversion if necessary
        #         os.remove(urdf_file_path)

        #         # Update progress bar after processing each file
        #         pbar.update(1)
        num_robots_processed = []
        elapsed_times = []
        start_time = time.time()

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

                # -- Timing data collection --------------------------------
                # If you only want to record every 50 robots, do:
                if i % 50 == 0:
                    current_time = time.time() - start_time  # elapsed seconds
                    num_robots_processed.append(i)          # e.g. how many done so far
                    elapsed_times.append(current_time)
                # ----------------------------------------------------------

        # Optionally, record final time if not a multiple of 50:
        if len(urdf_files) % 50 != 0:
            current_time = time.time() - start_time
            num_robots_processed.append(len(urdf_files))
            elapsed_times.append(current_time)

        # End timing for the total conversion process
        gen_end_time = time.time()
        gen_elapsed_time = gen_end_time - gen_start_time
        print(f"Total time taken for conversions six legged for generation {generation}: {gen_elapsed_time:.2f} seconds")

        fig = px.line(
            x=num_robots_processed,
            y=elapsed_times,
            markers=True,
            title="Number of Robots Processed vs. Elapsed Time"
        )
        fig.update_layout(
            xaxis_title="Number of Robots Processed",
            yaxis_title="Elapsed Time (seconds)"
        )

        # Save plot as an HTML file in the same directory
        output_file = "plot.html"
        fig.write_html(output_file)
        print(f"Plot saved to {output_file}")

        data = {"Number of Robots Processed": num_robots_processed, "Elapsed Time (seconds)": elapsed_times}
        df = pd.DataFrame(data)
        csv_filename = "plot_data_config_2_legged_comp.csv"
        df.to_csv(csv_filename, index=False)
        print(f"Plot data saved to {csv_filename}")

        simulation_app.close()
