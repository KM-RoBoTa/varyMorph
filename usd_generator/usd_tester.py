from isaacsim import SimulationApp
import time
import os

# URDF import, configuration, and simulation sample
kit = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.examples")
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
import omni.kit.commands
import omni.kit.usd


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = World()
        world.scene.add_default_ground_plane()

        # Acquire the URDF extension interface
        urdf_interface = _urdf.acquire_urdf_interface()

        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = False
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 0
        import_config.default_position_drive_damping = 0
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.distance_scale = 1

        # Define the specific URDF file path and the desired USDA file path
        urdf_file_path = "/home/gpuuser/GitHub/Eureka/isaacgymenvs/assets/urdf/anymal_c/urdf/anymal.urdf"
        usda_file_path = urdf_file_path.replace(".urdf", ".usda")
        
        # Start timing for the conversion process
        start_time = time.time()
        
        # Import the URDF and convert it to USDA
        result, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_file_path,
            import_config=import_config,
            dest_path=usda_file_path
        )

        # End timing for the conversion process
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Time taken for the conversion: {elapsed_time:.2f} seconds")
        return


if __name__ == "__main__":
    sample = HelloWorld()
    sample.setup_scene()
