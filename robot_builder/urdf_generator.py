
import os

class URDFCreator:
    def make_cylinder_link(self,link_name:str,link_height:float,radius:float,collision:bool,origin,inertia,mass,com):
    
        link=f"  <link name=\"{link_name}\"> "
        link+="\n"
        link+="    <visual>"
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\"  xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
        link+="\n"
        link+="       <geometry>"
        link+="\n"
        link+=f"           <cylinder length= \"{link_height}\" radius=\"{radius}\"/>"
        link+="\n"
        link+="       </geometry>"
        link+="\n"
        link+=f"    </visual> "
    
        if (collision):
          link+="\n"
          link+="    <collision> "
          link+="\n"
          link+=f"       <origin rpy=\"0 0 0\" xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
          link+="\n"
          link+="       <geometry>"
          link+="\n"
          link+=f"           <cylinder length= \"{link_height}\" radius=\"{radius}\"/>"
          link+="\n"
          link+="       </geometry>"
          link+="\n"
          link+="    </collision> "
          
        link+="\n"
        link+="    <inertial> "
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\" xyz=\"{com[0]} {com[1]} {com[2]}\"/>"
        link+="\n"
        link+=f"       <mass value=\"{mass}\"/>"
        link+="\n"
        link+=f"       <inertia ixx=\"{inertia[0]}\" ixy=\"{inertia[1]}\" ixz=\"{inertia[2]}\" iyy=\"{inertia[3]}\" iyz=\"{inertia[4]}\" izz=\"{inertia[5]}\"/>"
        link+="\n"
        link+="    </inertial> "
        link+="\n"
    
        link+="  </link>"
        link+="\n"
        link+="\n"
        return link
    
    
    def make_sphere_link(self,link_name:str,radius:float,collision:bool,origin,inertia,mass,com):
    
        link=f"  <link name=\"{link_name}\"> "
        link+="\n"
        link+="    <visual>"
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\"  xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
        link+="\n"
        link+="       <geometry>"
        link+="\n"
        link+=f"           <sphere radius=\"{radius}\"/>"
        link+="\n"
        link+="       </geometry>"
        link+="\n"
        link+=f"    </visual> "
    
        if (collision):
          link+="\n"
          link+="    <collision> "
          link+="\n"
          link+=f"       <origin rpy=\"0 0 0\" xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
          link+="\n"
          link+="       <geometry>"
          link+="\n"
          link+=f"           <sphere radius=\"{radius}\"/>"
          link+="\n"
          link+="       </geometry>"
          link+="\n"
          link+="    </collision> "
          
        link+="\n"
        link+="    <inertial> "
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\" xyz=\"{com[0]} {com[1]} {com[2]}\"/>"
        link+="\n"
        link+=f"       <mass value=\"{mass}\"/>"
        link+="\n"
        link+=f"       <inertia ixx=\"{inertia[0]}\" ixy=\"{inertia[1]}\" ixz=\"{inertia[2]}\" iyy=\"{inertia[3]}\" iyz=\"{inertia[4]}\" izz=\"{inertia[5]}\"/>"
        link+="\n"
        link+="    </inertial> "
        link+="\n"
    
        link+="  </link>"
        link+="\n"
        link+="\n"
        return link

    def make_box_link(self,link_name:str,size,collision:bool,origin,inertia,mass,com):
    
        link=f"  <link name=\"{link_name}\"> "
        link+="\n"
        link+="    <visual>"
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\"  xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
        link+="\n"
        link+="       <geometry>"
        link+="\n"
        link+=f"           <box size=\"{size[0]} {size[1]} {size[2]}\"/>" 
        link+="\n"
        link+="       </geometry>"
        link+="\n"
        link+=f"    </visual> "
    
        if (collision):
          link+="\n"
          link+="    <collision> "
          link+="\n"
          link+=f"       <origin rpy=\"0 0 0\" xyz=\"{origin[0]} {origin[1]} {origin[2]}\"/>"
          link+="\n"
          link+="       <geometry>"
          link+="\n"
          link+=f"           <box size=\"{size[0]} {size[1]} {size[2]}\"/>"
          link+="\n"
          link+="       </geometry>"
          link+="\n"
          link+="    </collision> "
          
        link+="\n"
        link+="    <inertial> "
        link+="\n"
        link+=f"       <origin rpy=\"0 0 0\" xyz=\"{com[0]} {com[1]} {com[2]}\"/>"
        link+="\n"
        link+=f"       <mass value=\"{mass}\"/>"
        link+="\n"
        link+=f"       <inertia ixx=\"{inertia[0]}\" ixy=\"{inertia[1]}\" ixz=\"{inertia[2]}\" iyy=\"{inertia[3]}\" iyz=\"{inertia[4]}\" izz=\"{inertia[5]}\"/>"
        link+="\n"
        link+="    </inertial> "
        link+="\n"
    
        link+="  </link>"
        link+="\n"
        link+="\n"
        return link
    
    
    def make_joint(self,joint_name,type,parent,child,joint_pos,axis):
        
        if type=="fixed":
          joint=f"  <joint name=\"{joint_name}\" type=\"{type}\" dont_collapse=\"true\">"
          joint+="\n"
          joint+=f"    <origin rpy=\"0 0 0\"  xyz=\"{joint_pos[0]} {joint_pos[1]} {joint_pos[2]}\"/>"
          joint+="\n"
          joint+=f"    <parent link=\"{parent}\"/>"
          joint+="\n"
          joint+=f"    <child link=\"{child}\"/>"
          joint+="\n"
          joint+="  </joint>"
          joint+="\n"
          joint+="\n"
        else:
          joint=f"  <joint name=\"{joint_name}\" type=\"{type}\">"
          joint+="\n"
          joint+=f"    <origin rpy=\"0 0 0\"  xyz=\"{joint_pos[0]} {joint_pos[1]} {joint_pos[2]}\"/>"
          joint+="\n"
          joint+=f"    <parent link=\"{parent}\"/>"
          joint+="\n"
          joint+=f"    <child link=\"{child}\"/>"
          joint+="\n"
          joint+=f"    <axis xyz=\"{axis[0]} {axis[1]} {axis[2]}\"/>"
          joint+="\n"
          joint+="    <dynamics damping=\"0\" friction=\"0\"/>"
          joint+="\n"
          joint+="    <limit effort=\"60\" lower=\"-3.141592653\" upper=\"3.141592653\" velocity=\"52.4\"/>"
          joint+="\n"
          joint+="  </joint>"
          joint+="\n"
          joint+="\n"
    
        return joint
    
'''
if __name__ == '__main__':
    
  urdf=urdf_creator()
  text_file = open("sample.urdf", "w")
      
  content= " <robot name=\"dog3_description\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">  "
  content+= "\n"
  
  mass=8
  inertia= [0,8,13,7,7,3]
  com= [12,8,13]
  link_height=0.3
  origin=[0,0,link_height/2]
  radius= 0.08


  content+=urdf.make_box_link(link_name="base",size=[0.01,0.01,0.01],collision=False,origin=[0.0, 0.0, 0.0],inertia=[0.0, 0.0, 0.0,0.0,0.0,0.0],mass=0.0,com=[0.0, 0.0, 0.0] ) #or not(type == "sphere")
  content+=urdf.make_box_link(link_name="trunk",size=[0.23,0.18,0.14],collision=False,origin=[0.0, 0.0, 0.0],inertia=[0.01683993, 8.3902e-05, 0.000597679, 0.106579028, 2.5134e-05,0.064713601],mass=1.713,com=[0.012731, 0.002186, 0.000515] )

  #Front right limb
  content+=urdf.make_sphere_link(link_name="FR_hip",radius=0.042,collision=False,origin=[-0.022191, -0.015144, -1.5e-05],inertia=[0.002446735, 0.00059805, 1.945e-06, 0.003925876, -1.284e-06, 0.004148145],mass=1.993,com=[-0.022191, -0.015144, -1.5e-05])
  content+=urdf.make_cylinder_link(link_name="FR_thigh",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_cylinder_link(link_name="FR_calf",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_sphere_link(link_name="FR_hip",radius=0.0165,collision=True,origin=[-0.0, -0.0, -0.0],inertia=[1.4e-05, 0.0, 0.0, 1.4e-05, 0.0, 1.4e-05],mass=0.12,com=[-0.0, -0.0, 0.0])

  #Front left limb
  content+=urdf.make_sphere_link(link_name="FL_hip",radius=0.042,collision=False,origin=[-0.022191, -0.015144, -1.5e-05],inertia=[0.002446735, 0.00059805, 1.945e-06, 0.003925876, -1.284e-06, 0.004148145],mass=1.993,com=[-0.022191, -0.015144, -1.5e-05])
  content+=urdf.make_cylinder_link(link_name="FL_thigh",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_cylinder_link(link_name="FL_calf",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_sphere_link(link_name="FL_hip",radius=0.0165,collision=True,origin=[-0.0, -0.0, -0.0],inertia=[1.4e-05, 0.0, 0.0, 1.4e-05, 0.0, 1.4e-05],mass=0.12,com=[-0.0, -0.0, 0.0])

  #Rear right limb
  content+=urdf.make_sphere_link(link_name="RR_hip",radius=0.042,collision=False,origin=[-0.022191, -0.015144, -1.5e-05],inertia=[0.002446735, 0.00059805, 1.945e-06, 0.003925876, -1.284e-06, 0.004148145],mass=1.993,com=[-0.022191, -0.015144, -1.5e-05])
  content+=urdf.make_cylinder_link(link_name="RR_thigh",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_cylinder_link(link_name="RR_calf",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_sphere_link(link_name="RR_hip",radius=0.0165,collision=True,origin=[-0.0, -0.0, -0.0],inertia=[1.4e-05, 0.0, 0.0, 1.4e-05, 0.0, 1.4e-05],mass=0.12,com=[-0.0, -0.0, 0.0])

  #Rear left limb
  content+=urdf.make_sphere_link(link_name="RL_hip",radius=0.042,collision=False,origin=[-0.022191, -0.015144, -1.5e-05],inertia=[0.002446735, 0.00059805, 1.945e-06, 0.003925876, -1.284e-06, 0.004148145],mass=1.993,com=[-0.022191, -0.015144, -1.5e-05])
  content+=urdf.make_cylinder_link(link_name="RL_thigh",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_cylinder_link(link_name="RL_calf",link_height=0.24,radius=0.028,collision=False,origin=[-0.0, -0.0, -0.12],inertia=[0.004173855, -1.0284e-05, -0.000318874, 0.004343802, -0.000109233, 0.000340136],mass=0.639,com=[-0.00, -0.0, -0.0])
  content+=urdf.make_sphere_link(link_name="RL_hip",radius=0.0165,collision=True,origin=[-0.0, -0.0, -0.0],inertia=[1.4e-05, 0.0, 0.0, 1.4e-05, 0.0, 1.4e-05],mass=0.12,com=[-0.0, -0.0, 0.0])


  ###############################Joints################################
  content+=urdf.make_joint(joint_name="floating_base",type="fixed",parent="base",child="trunk",joint_pos=[0.0,0.0,0.0],axis=[0,0,0])
  content+=urdf.make_joint(joint_name="FL_hip_joint",type="revolute",parent="trunk",child="FL_hip",joint_pos=[0.149, 0.056, -0.0235],axis=[1,0,0])
  content+=urdf.make_joint(joint_name="FR_hip_joint",type="revolute",parent="trunk",child="FR_hip",joint_pos=[0.149, -0.056, -0.0235],axis=[1,0,0])
  content+=urdf.make_joint(joint_name="RL_hip_joint",type="revolute",parent="trunk",child="RL_hip",joint_pos=[-0.149, 0.056, -0.0235],axis=[1,0,0])
  content+=urdf.make_joint(joint_name="RR_hip_joint",type="revolute",parent="trunk",child="RR_hip",joint_pos=[-0.149, -0.056, -0.0235],axis=[1,0,0])

  content+=urdf.make_joint(joint_name="FL_thigh_joint",type="revolute",parent="FL_hip",child="FL_thigh",joint_pos=[0, 0.057083, 0],axis=[0,1,0])
  content+=urdf.make_joint(joint_name="FR_thigh_joint",type="revolute",parent="FR_hip",child="FR_thigh",joint_pos=[0, -0.057083, 0],axis=[0,1,0]) 
  content+=urdf.make_joint(joint_name="RL_thigh_joint",type="revolute",parent="RL_hip",child="RL_thigh",joint_pos=[0, 0.057083, 0],axis=[0,1,0])
  content+=urdf.make_joint(joint_name="RR_thigh_joint",type="revolute",parent="RR_hip",child="RR_thigh",joint_pos=[0, -0.057083, 0],axis=[0,1,0]) 

  content+=urdf.make_joint(joint_name="FL_calf_joint",type="revolute",parent="FL_thigh",child="FL_calf",joint_pos=[0, 0, -0.25],axis=[0,1,0])
  content+=urdf.make_joint(joint_name="FR_calf_joint",type="revolute",parent="FR_thigh",child="FR_calf",joint_pos=[0, 0, -0.25],axis=[0,1,0]) 
  content+=urdf.make_joint(joint_name="RL_calf_joint",type="revolute",parent="RL_thigh",child="RL_calf",joint_pos=[0, 0, -0.25],axis=[0,1,0])
  content+=urdf.make_joint(joint_name="RR_calf_joint",type="revolute",parent="RR_thigh",child="RR_calf",joint_pos=[0, 0, -0.25],axis=[0,1,0]) 

  content+=urdf.make_joint(joint_name="FL_foot_joint",type="fixed",parent="FL_calf",child="FL_foot",joint_pos=[0, 0, -0.25],axis=[0,0,0])
  content+=urdf.make_joint(joint_name="FR_foot_joint",type="fixed",parent="FR_calf",child="FR_foot",joint_pos=[0, 0, -0.25],axis=[0,0,0]) 
  content+=urdf.make_joint(joint_name="RL_foot_joint",type="fixed",parent="RL_calf",child="RL_foot",joint_pos=[0, 0, -0.25],axis=[0,0,0])
  content+=urdf.make_joint(joint_name="RR_foot_joint",type="fixed",parent="RR_calf",child="RR_foot",joint_pos=[0, 0, -0.25],axis=[0,0,0]) 

  text_file.write(content)
  text_file.close()
'''