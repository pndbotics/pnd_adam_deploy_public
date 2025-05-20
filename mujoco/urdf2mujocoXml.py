import xml.etree.ElementTree as ET
import mujoco as mj

# add free joint
ADD_FREE_JOINT = False

def mjcf_from_urdf(urdf_path):
    # read urdf
    urdf_tree = ET.parse(urdf_path)
    urdf_root=urdf_tree.getroot()

    mujoco = ET.SubElement(urdf_root, "mujoco")
    compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("meshdir", "assets")
    compiler.set("balanceinertia", "true")
    compiler.set("discardvisual", "false")
    
    # get new urdf
    new_urdf = ET.ElementTree(urdf_root)
    new_urdf_save_path = urdf_path.replace(".urdf", "_mujoco.urdf")
    new_urdf.write(new_urdf_save_path)
    print("New urdf is saved at: ", new_urdf_save_path)
    
    # load urdf into mujoco
    model =  mj.MjModel.from_xml_path(new_urdf_save_path)
    # save mjcf
    mjcf_path = urdf_path.replace(".urdf", ".xml")
    mj.mj_saveLastXML( mjcf_path,model,)
    print("Mujoco model is saved at: ", mjcf_path)
    
    # poseprocess mjcf
    # add base body
    mjcf_xml = ET.parse(mjcf_path)
    mjcf_root = mjcf_xml.getroot()
    
    
    # rename worldbody to body
    old_worldbody = mjcf_root.find("worldbody")
    pelvis = old_worldbody.find("body")
    pelvis.set("pos", "0 0 0.95")
    
    # add freejoint to base body
    if ADD_FREE_JOINT:
        freejoint = ET.SubElement(old_worldbody, "freejoint")

    site = ET.SubElement(pelvis, "site")
    site.set("name", "imu")
    site.set("pos", "0 0 0")
    
    # remove old worldbody
    # add new worldbody
    if ADD_FREE_JOINT:
        worldbody = ET.SubElement(mjcf_root, "worldbody")
        # add oldbody to worldbody
        worldbody.append(old_worldbody)
        mjcf_root.remove(old_worldbody)
    
    # get all the joint
    joints = mjcf_root.findall(".//joint")
    joint_names = []
    joint_torque_limits_lower = []
    joint_torque_limits_upper = []
    for joint in joints:
        joint_type = joint.get("type")
        if joint_type is not None:
            if joint_type == "fixed":
                continue
            if joint_type == "free":
                continue
        joint_names.append(joint.get("name"))
        actuatorfrcrange=joint.get("actuatorfrcrange")
        t_l=float(actuatorfrcrange.split(" ")[0])
        t_u=float(actuatorfrcrange.split(" ")[1])
        joint_torque_limits_lower.append(t_l)
        joint_torque_limits_upper.append(t_u)
        print(f"joint {joint.get('name')} torque limits: {t_l} {t_u}")

    # add actuators
    actuators = ET.SubElement(mjcf_root, "actuator")
    for joint_name,joint_l,joint_u in zip(joint_names,joint_torque_limits_lower,joint_torque_limits_upper):
        motor=ET.SubElement(actuators, "motor")
        motor.set("gear", "1")
        motor.set("joint", joint_name)
        motor.set("name", joint_name)
        motor.set("ctrllimited", "true")
        motor.set("ctrlrange", f"{joint_l} {joint_u}")
    sensors = ET.SubElement(mjcf_root, "sensor")

    for joint_name in joint_names:
        jointvel = ET.SubElement(sensors, "jointpos")
        jointvel.set("name", f"pos_{joint_name}")
        jointvel.set("joint", joint_name)

    for joint_name in joint_names:
        jointvel = ET.SubElement(sensors, "jointvel")
        jointvel.set("name", f"vel_{joint_name}")
        jointvel.set("joint", joint_name)

    for joint_name in joint_names:
        jointvel = ET.SubElement(sensors, "jointactuatorfrc")
        jointvel.set("name", f"force_{joint_name}")
        jointvel.set("joint", joint_name)

            
    #add imu sensor   
    accelerometer = ET.SubElement(sensors, "accelerometer")
    accelerometer.set("name", "BodyAcc")
    accelerometer.set("site", "imu")
    gyro = ET.SubElement(sensors, "gyro")
    gyro.set("name", "BodyGyro")
    gyro.set("site", "imu")
    framepos = ET.SubElement(sensors, "framepos")
    framepos.set("name", "BodyPos")
    framepos.set("objtype", "site")
    framepos.set("objname", "imu")
    framevel = ET.SubElement(sensors, "framelinvel")
    framevel.set("name", "BodyVel")
    framevel.set("objtype", "site")
    framevel.set("objname", "imu")
    
    framequat = ET.SubElement(sensors, "framequat")
    framequat.set("name", "BodyQuat")
    framequat.set("objtype", "site")
    framequat.set("objname", "imu")

    # save new mjcf with formating
    mjcf_xml.write(mjcf_path)
    
    print("Mujoco model is saved at: ", mjcf_path)
    
    # write new urdf
if __name__ == "__main__":
    mjcf_from_urdf("./model/adam_lite/adam_lite.urdf")

