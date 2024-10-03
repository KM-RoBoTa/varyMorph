from pxr import Usd, UsdGeom, Sdf, Gf

def create_usd_robot(output_file):
    # Create a new USD stage
    stage = Usd.Stage.CreateNew(output_file)

    # Define the root Xform for the robot
    root_xform = UsdGeom.Xform.Define(stage, '/Robot')

    # Function to create a link
    def create_link(name, size, parent_path):
        link_path = parent_path.AppendChild(name)
        link_xform = UsdGeom.Xform.Define(stage, link_path)
        link_geom = UsdGeom.Cube.Define(stage, link_path.AppendChild("geom"))
        link_geom.GetSizeAttr().Set(size)
        link_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, size / 2.0))
        return link_xform

    # Function to create a joint (simple representation)
    def create_joint(name, parent_path):
        joint_path = parent_path.AppendChild(name)
        joint_xform = UsdGeom.Xform.Define(stage, joint_path)
        return joint_xform

    # Create links
    link1 = create_link('Link1', 1.0, root_xform.GetPath())
    link2 = create_link('Link2', 0.5, root_xform.GetPath())

    # Position Link2 relative to Link1
    link2.AddTranslateOp().Set(Gf.Vec3f(0, 1.0, 0))

    # Create a joint between Link1 and Link2
    joint = create_joint('Joint1', root_xform.GetPath())
    joint.AddTranslateOp().Set(Gf.Vec3f(0, 1.0, 0))

    # Set joint properties (example, extend as needed)
    joint_prim = stage.GetPrimAtPath(joint.GetPath())
    joint_prim.GetReferences().AddReference('/Robot/Link2')

    # Save the USD stage to the file
    stage.GetRootLayer().Save()

# Usage
output_file = 'simple_robot.usd'
create_usd_robot(output_file)
print(f"USD file created at: {output_file}")
