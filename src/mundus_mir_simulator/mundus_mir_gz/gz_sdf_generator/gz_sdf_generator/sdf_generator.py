import yaml
import os
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Getting content of yaml file
def load_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

def prettify(elem):
    """Return a pretty-printed XML string for the Element."""
    from xml.dom import minidom
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

class SDF_Generator(Node):

    def __init__(self):
        super().__init__("sdf_generator")
        self.declare_parameter("config_path", "/tmp")
        self.declare_parameter("launch_file", "generated_mundus_mir_simple_world")
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.launch_file = self.get_parameter("launch_file").get_parameter_value().string_value
        self.generate_sdf()

    def generate_sdf(self):
        config = load_yaml_file(self.config_path)
        root = ET.Element("sdf", version="1.9")
        model = ET.SubElement(root, "model", name=config["model_name"], canonical_link=config["model_name"] + "_link")


        # Main model link
        main_model_link = ET.SubElement(model, "link", name=config["model_name"] + "_link")
        main_model_link.insert(0, ET.Comment("Main model link"))
        main_model_pose = ET.SubElement(main_model_link, "pose", relative_to="__model__")
        main_model_pose.text = "0.0 0.0 0.0 0.0 0.0 0.0"
        main_model_visual = ET.SubElement(main_model_link, "visual", name=config["model_name"] + "_visual")
        main_model_visual_pose = ET.SubElement(main_model_visual, "pose", relative_to=config["model_name"] + "_link")
        main_model_visual_pose.text = config["model_visual_pose"]
        main_mode_visual_geometry = ET.SubElement(main_model_visual, "geometry")
        main_model_visual_mesh = ET.SubElement(main_mode_visual_geometry, "mesh")
        main_model_visual_uri = ET.SubElement(main_model_visual_mesh, "uri")
        main_model_visual_uri.text = "model://models/dynamic_assets/" + config["model_name"] + "/meshes/" + config["model_name"] + ".dae"
        main_model_visual_scale = ET.SubElement(main_model_visual_mesh, "scale")
        main_model_visual_scale.text = "1 1 1"
        main_model_inertial = ET.SubElement(main_model_link, "inertial")
        main_model_inertial_mass = ET.SubElement(main_model_inertial, "mass")
        main_model_inertial_mass.text = str(config["mass"])
        main_model_inertial_pose = ET.SubElement(main_model_inertial, "pose")
        main_model_inertial_pose.text = "0.0 0.0 0.0 0.0 0.0 0.0"
        main_model_inertial_inertia  = ET.SubElement(main_model_inertial, "inertia")
        main_model_inertial_inertia_ixx = ET.SubElement(main_model_inertial_inertia, "ixx")
        main_model_inertial_inertia_ixx.text = str(config["ixx"])
        main_model_inertial_inertia_ixy = ET.SubElement(main_model_inertial_inertia, "ixy")
        main_model_inertial_inertia_ixy.text = str(config["ixy"])
        main_model_inertial_inertia_ixz = ET.SubElement(main_model_inertial_inertia, "ixz")
        main_model_inertial_inertia_ixz.text = str(config["ixz"])
        main_model_inertial_inertia_iyy = ET.SubElement(main_model_inertial_inertia, "iyy")
        main_model_inertial_inertia_iyy.text = str(config["iyy"])
        main_model_inertial_inertia_iyz = ET.SubElement(main_model_inertial_inertia, "iyz")
        main_model_inertial_inertia_iyz.text = str(config["iyz"])
        main_model_inertial_inertia_izz = ET.SubElement(main_model_inertial_inertia, "izz")
        main_model_inertial_inertia_izz.text = str(config["izz"])

        # Added mass
        main_model_inertial_fluid_mass = ET.SubElement(main_model_inertial, "fluid_added_mass")
        main_model_inertial_fluid_mass_xx = ET.SubElement(main_model_inertial_fluid_mass, "xx")
        main_model_inertial_fluid_mass_xx.text = config["xDotU"]
        main_model_inertial_fluid_mass_yy = ET.SubElement(main_model_inertial_fluid_mass, "yy")
        main_model_inertial_fluid_mass_yy.text = config["yDotV"]
        main_model_inertial_fluid_mass_zz = ET.SubElement(main_model_inertial_fluid_mass, "zz")
        main_model_inertial_fluid_mass_zz.text = config["zDotW"]
        main_model_inertial_fluid_mass_pp = ET.SubElement(main_model_inertial_fluid_mass, "pp")
        main_model_inertial_fluid_mass_pp.text = config["kDotP"]
        main_model_inertial_fluid_mass_qq = ET.SubElement(main_model_inertial_fluid_mass, "qq")
        main_model_inertial_fluid_mass_qq.text = config["mDotQ"]
        main_model_inertial_fluid_mass_rr = ET.SubElement(main_model_inertial_fluid_mass, "rr")
        main_model_inertial_fluid_mass_rr.text = config["nDotR"]

        # Collision box for bouyancy
        main_model_collision = ET.SubElement(main_model_link, "collision", name=config["model_name"] + "_collision")
        main_model_collision.insert(0, ET.Comment("Collision box for bouyancy"))
        main_model_collision_pose = ET.SubElement(main_model_collision, "pose", relative_to=config["model_name"] + "_link")
        main_model_collision_pose.text = "0.0 0.0 0.1 0.0 0.0 0.0"
        main_model_collision_geometry = ET.SubElement(main_model_collision, "geometry")
        main_model_collision_box = ET.SubElement(main_model_collision_geometry, "box")
        main_model_collision_size = ET.SubElement(main_model_collision_box, "size")
        main_model_collision_size.text = config["collision_size"]

        # Ground Truth Position 
        odometry_plugin = ET.SubElement(model, "plugin", name="gz::sim::systems::OdometryPublisher", filename="gz-sim-odometry-publisher-system")
        odometry_plugin.append(ET.Comment("Ground Truth Position"))
        odom_plugin_frame = ET.SubElement(odometry_plugin, "frame")
        odom_plugin_frame.text = "world_flu"
        odom_plugin_robot_base_frame = ET.SubElement(odometry_plugin, "robot_base_frame")
        odom_plugin_robot_base_frame.text = config["model_name"] + "_link"
        odom_plugin_odom_topic = ET.SubElement(odometry_plugin, "odom_topic")
        odom_plugin_odom_topic.text = config["model_name"] + "/odometry_flu/gt"
        odom_plugin_dimensions = ET.SubElement(odometry_plugin, "dimensions")
        odom_plugin_dimensions.text = "3"

        # Blueye Light
        light_link = ET.SubElement(model, "link", name=config["model_name"] + "_light_link")
        light_link.append(ET.Comment("Blueye Light Link"))
        light_link_pose = ET.SubElement(light_link, "pose", relative_to=config["model_name"] + "_link")
        light_link_pose.text = "0.0 0.0 0.0 0.0 0.0 0.0"
        light_link_light = ET.SubElement(light_link, "light", name=config["model_name"] + "_light", type="spot")
        light_link_light_pose = ET.SubElement(light_link_light, "pose")
        light_link_light_pose.text = "0.0 0.0 0.0 0.0 0.0 0.0"
        light_link_light_diffuse = ET.SubElement(light_link_light, "diffuse")
        light_link_light_diffuse.text = "1.0 1.0 1.0 1.0"
        light_link_light_specular = ET.SubElement(light_link_light, "specular")
        light_link_light_specular.text = "0.5 0.5 0.5 1.0"
        light_link_light_attenuation = ET.SubElement(light_link_light, "attenuation")
        light_link_light_attenuation_range = ET.SubElement(light_link_light_attenuation, "range")
        light_link_light_attenuation_range.text = "20.0"
        light_link_light_attenuation_constant = ET.SubElement(light_link_light_attenuation, "constant")
        light_link_light_attenuation_constant.text = "0.5"
        light_link_light_attenuation_linear = ET.SubElement(light_link_light_attenuation, "linear")
        light_link_light_attenuation_linear.text = "1.0"
        light_link_light_attenuation_quadratic = ET.SubElement(light_link_light_attenuation, "quadratic")   
        light_link_light_attenuation_quadratic.text = "0.3"
        light_link_light_direction = ET.SubElement(light_link_light, "direction")
        light_link_light_direction.text = "1.0 0.0 0.0"
        light_link_light_cast_shadow = ET.SubElement(light_link_light, "cast_shadows")
        light_link_light_cast_shadow.text = "1"
        light_link_light_spot = ET.SubElement(light_link_light, "spot")
        light_link_light_spot_inner_angle = ET.SubElement(light_link_light_spot, "inner_angle")
        light_link_light_spot_inner_angle.text = "1.0"
        light_link_light_spot_outer_angle = ET.SubElement(light_link_light_spot, "outer_angle")
        light_link_light_spot_outer_angle.text = "1.0"
        light_link_light_spot_falloff = ET.SubElement(light_link_light_spot, "falloff")
        light_link_light_spot_falloff.text = "1.0"
        light_link_inertia = ET.SubElement(light_link, "inertial")
        light_link_inertia_mass = ET.SubElement(light_link_inertia, "mass")
        light_link_inertia_mass.text = "0.0001"

        # Blueye Light Joint
        light_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_light_joint", type="fixed")
        light_joint.append(ET.Comment("Blueye Light Joint"))
        light_joint_child = ET.SubElement(light_joint, "child")
        light_joint_child.text = config["model_name"] + "_light_link"
        light_joint_parent = ET.SubElement(light_joint, "parent")
        light_joint_parent.text = config["model_name"] + "_link"

        # Sensors
        if ("imu" in config and config["imu"]):

            # IMU Sensor Link
            imu_link = ET.SubElement(model, "link", name=config["model_name"] + "_imu_link")
            imu_link.insert(0, ET.Comment("IMU Sensor Link"))
            imu_link_pose = ET.SubElement(imu_link, "pose", relative_to=config["model_name"] + "_link")
            imu_link_pose.text = config["imu_pose"]
            imu_sensor = ET.SubElement(imu_link, "sensor", name=config["model_name"] + "_imu", type="imu")
            imu_sensor_update_rate = ET.SubElement(imu_sensor, "update_rate")
            imu_sensor_update_rate.text = str(config["imu_update_rate"])
            imu_sensor_topic = ET.SubElement(imu_sensor, "topic")
            imu_sensor_topic.text = config["imu_topic"]
            imu_sensor_imu = ET.SubElement(imu_sensor, "imu")
            imu_sensor_imu_linear = ET.SubElement(imu_sensor_imu, "linear_acceleration")
            imu_sensor_imu_linear_x = ET.SubElement(imu_sensor_imu_linear, "x")
            imu_sensor_imu_linear_x_noise = ET.SubElement(imu_sensor_imu_linear_x, "noise", type="gaussian")
            imu_sensor_imu_linear_x_noise_mean = ET.SubElement(imu_sensor_imu_linear_x_noise, "mean")
            imu_sensor_imu_linear_x_noise_mean.text = str(config["imu_linear_x_mean"])
            imu_sensor_imu_linear_x_noise_stddev = ET.SubElement(imu_sensor_imu_linear_x_noise, "stddev")
            imu_sensor_imu_linear_x_noise_stddev.text = str(config["imu_linear_x_stddev"])
            imu_sensor_imu_linear_x_noise_bias_mean = ET.SubElement(imu_sensor_imu_linear_x_noise, "bias_mean")
            imu_sensor_imu_linear_x_noise_bias_mean.text = str(config["imu_linear_x_bias_mean"])
            imu_sensor_imu_linear_x_noise_bias_stddev = ET.SubElement(imu_sensor_imu_linear_x_noise, "bias_stddev")
            imu_sensor_imu_linear_x_noise_bias_stddev.text = str(config["imu_linear_x_bias_stddev"])
            imu_sensor_imu_linear_y = ET.SubElement(imu_sensor_imu_linear, "y")
            imu_sensor_imu_linear_y_noise = ET.SubElement(imu_sensor_imu_linear_y, "noise", type="gaussian")
            imu_sensor_imu_linear_y_noise_mean = ET.SubElement(imu_sensor_imu_linear_y_noise, "mean")
            imu_sensor_imu_linear_y_noise_mean.text = str(config["imu_linear_y_mean"])
            imu_sensor_imu_linear_y_noise_stddev = ET.SubElement(imu_sensor_imu_linear_y_noise, "stddev")
            imu_sensor_imu_linear_y_noise_stddev.text = str(config["imu_linear_y_stddev"])
            imu_sensor_imu_linear_y_noise_bias_mean = ET.SubElement(imu_sensor_imu_linear_y_noise, "bias_mean")
            imu_sensor_imu_linear_y_noise_bias_mean.text = str(config["imu_linear_y_bias_mean"])
            imu_sensor_imu_linear_y_noise_bias_stddev = ET.SubElement(imu_sensor_imu_linear_y_noise, "bias_stddev")
            imu_sensor_imu_linear_y_noise_bias_stddev.text = str(config["imu_linear_y_bias_stddev"])
            imu_sensor_imu_linear_z = ET.SubElement(imu_sensor_imu_linear, "z")
            imu_sensor_imu_linear_z_noise = ET.SubElement(imu_sensor_imu_linear_z, "noise", type="gaussian")
            imu_sensor_imu_linear_z_noise_mean = ET.SubElement(imu_sensor_imu_linear_z_noise, "mean")
            imu_sensor_imu_linear_z_noise_mean.text = str(config["imu_linear_z_mean"])
            imu_sensor_imu_linear_z_noise_stddev = ET.SubElement(imu_sensor_imu_linear_z_noise, "stddev")
            imu_sensor_imu_linear_z_noise_stddev.text = str(config["imu_linear_z_stddev"])
            imu_sensor_imu_linear_z_noise_bias_mean = ET.SubElement(imu_sensor_imu_linear_z_noise, "bias_mean")
            imu_sensor_imu_linear_z_noise_bias_mean.text = str(config["imu_linear_z_bias_mean"])
            imu_sensor_imu_linear_z_noise_bias_stddev = ET.SubElement(imu_sensor_imu_linear_z_noise, "bias_stddev")
            imu_sensor_imu_linear_z_noise_bias_stddev.text = str(config["imu_linear_z_bias_stddev"])
            imu_sensor_imu_angular = ET.SubElement(imu_sensor_imu, "angular_velocity")
            imu_sensor_imu_angular_x = ET.SubElement(imu_sensor_imu_angular, "x")
            imu_sensor_imu_angular_x_noise = ET.SubElement(imu_sensor_imu_angular_x, "noise", type="gaussian")
            imu_sensor_imu_angular_x_noise_mean = ET.SubElement(imu_sensor_imu_angular_x_noise, "mean")
            imu_sensor_imu_angular_x_noise_mean.text = str(config["imu_angular_x_mean"])
            imu_sensor_imu_angular_x_noise_stddev = ET.SubElement(imu_sensor_imu_angular_x_noise, "stddev")
            imu_sensor_imu_angular_x_noise_stddev.text = str(config["imu_angular_x_stddev"])
            imu_sensor_imu_angular_x_noise_bias_mean = ET.SubElement(imu_sensor_imu_angular_x_noise, "bias_mean")
            imu_sensor_imu_angular_x_noise_bias_mean.text = str(config["imu_angular_x_bias_mean"])
            imu_sensor_imu_angular_x_noise_bias_stddev = ET.SubElement(imu_sensor_imu_angular_x_noise, "bias_stddev")
            imu_sensor_imu_angular_x_noise_bias_stddev.text = str(config["imu_angular_x_bias_stddev"])
            imu_sensor_imu_angular_y = ET.SubElement(imu_sensor_imu_angular, "y")
            imu_sensor_imu_angular_y_noise = ET.SubElement(imu_sensor_imu_angular_y, "noise", type="gaussian")
            imu_sensor_imu_angular_y_noise_mean = ET.SubElement(imu_sensor_imu_angular_y_noise, "mean")
            imu_sensor_imu_angular_y_noise_mean.text = str(config["imu_angular_y_mean"])
            imu_sensor_imu_angular_y_noise_stddev = ET.SubElement(imu_sensor_imu_angular_y_noise, "stddev")
            imu_sensor_imu_angular_y_noise_stddev.text = str(config["imu_angular_y_stddev"])
            imu_sensor_imu_angular_y_noise_bias_mean = ET.SubElement(imu_sensor_imu_angular_y_noise, "bias_mean")
            imu_sensor_imu_angular_y_noise_bias_mean.text = str(config["imu_angular_y_bias_mean"])
            imu_sensor_imu_angular_y_noise_bias_stddev = ET.SubElement(imu_sensor_imu_angular_y_noise, "bias_stddev")
            imu_sensor_imu_angular_y_noise_bias_stddev.text = str(config["imu_angular_y_bias_stddev"])
            imu_sensor_imu_angular_z = ET.SubElement(imu_sensor_imu_angular, "z")
            imu_sensor_imu_angular_z_noise = ET.SubElement(imu_sensor_imu_angular_z, "noise", type="gaussian")
            imu_sensor_imu_angular_z_noise_mean = ET.SubElement(imu_sensor_imu_angular_z_noise, "mean")
            imu_sensor_imu_angular_z_noise_mean.text = str(config["imu_angular_z_mean"])
            imu_sensor_imu_angular_z_noise_stddev = ET.SubElement(imu_sensor_imu_angular_z_noise, "stddev")
            imu_sensor_imu_angular_z_noise_stddev.text = str(config["imu_angular_z_stddev"])
            imu_sensor_imu_angular_z_noise_bias_mean = ET.SubElement(imu_sensor_imu_angular_z_noise, "bias_mean")
            imu_sensor_imu_angular_z_noise_bias_mean.text = str(config["imu_angular_z_bias_mean"])
            imu_sensor_imu_angular_z_noise_bias_stddev = ET.SubElement(imu_sensor_imu_angular_z_noise, "bias_stddev")
            imu_sensor_imu_angular_z_noise_bias_stddev.text = str(config["imu_angular_z_bias_stddev"])
            imu_sensor_always_on = ET.SubElement(imu_sensor, "always_on")
            imu_sensor_always_on.text = "1"
            imu_link_inertia = ET.SubElement(imu_link, "inertial")
            imu_link_inertia_mass = ET.SubElement(imu_link_inertia, "mass")
            imu_link_inertia_mass.text = "0.0001"

            # IMU Joint
            imu_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_imu_joint", type="fixed")
            imu_joint.insert(0, ET.Comment("IMU Joint"))
            imu_joint_child = ET.SubElement(imu_joint, "child")
            imu_joint_child.text = config["model_name"] + "_imu_link"
            imu_joint_parent = ET.SubElement(imu_joint, "parent")
            imu_joint_parent.text = config["model_name"] + "_link"


        if ("usbl" in config and config["usbl"]):
            pass

        if ("camera_1" in config and config["camera_1"]):
            # Camera Link
            camera_1_link = ET.SubElement(model, "link", name=config["model_name"] + "_camera_1_link")
            camera_1_link.append(ET.Comment("Camera 1 Link"))
            camera_1_link_pose = ET.SubElement(camera_1_link, "pose", relative_to=config["model_name"] + "_link")
            camera_1_link_pose.text = config["camera_1_pose"]
            camera_1_link_sensor = ET.SubElement(camera_1_link, "sensor", name=config["model_name"] + "_camera_1", type="camera")
            camera_1_link_sensor_camera = ET.SubElement(camera_1_link_sensor, "camera")
            camera_1_link_sensor_camera_image = ET.SubElement(camera_1_link_sensor_camera, "image")
            camera_1_link_sensor_camera_image_width = ET.SubElement(camera_1_link_sensor_camera_image, "width")
            camera_1_link_sensor_camera_image_width.text = str(config["camera_1_width"])
            camera_1_link_sensor_camera_image_height = ET.SubElement(camera_1_link_sensor_camera_image, "height")
            camera_1_link_sensor_camera_image_height.text = str(config["camera_1_height"])
            camera_1_link_sensor_camera_fov = ET.SubElement(camera_1_link_sensor_camera, "horizontal_fov")
            camera_1_link_sensor_camera_fov.text = str(config["camera_1_fov"])
            camera_1_link_sensor_camera_clip = ET.SubElement(camera_1_link_sensor_camera, "clip")
            camera_1_link_sensor_camera_clip_far = ET.SubElement(camera_1_link_sensor_camera_clip, "far")
            camera_1_link_sensor_camera_clip_far.text = str(config["camera_1_clip_far"])
            camera_1_link_sensor_camera_clip_near = ET.SubElement(camera_1_link_sensor_camera_clip, "near")
            camera_1_link_sensor_camera_clip_near.text = str(config["camera_1_clip_near"])
            camera_1_link_sensor_always_on = ET.SubElement(camera_1_link_sensor, "always_on")
            camera_1_link_sensor_always_on.text = "1"
            camera_1_link_sensor_update_rate = ET.SubElement(camera_1_link_sensor, "update_rate")
            camera_1_link_sensor_update_rate.text = str(config["camera_1_update_rate"])
            camera_1_link_sensor_topic = ET.SubElement(camera_1_link_sensor, "topic")
            camera_1_link_sensor_topic.text = config["camera_1_topic"]
            camera_1_link_sensor_camera_info_topic = ET.SubElement(camera_1_link_sensor_camera, "camera_info_topic")
            camera_1_link_sensor_camera_info_topic.text = config["camera_1_info_topic"]
            camera_1_link_inertia = ET.SubElement(camera_1_link, "inertial")
            camera_1_link_inertia_mass = ET.SubElement(camera_1_link_inertia, "mass")
            camera_1_link_inertia_mass.text = "0.0001"
            camera_1_link_sensor_visualize = ET.SubElement(camera_1_link_sensor, "visualize")
            camera_1_link_sensor_visualize.text = "1"

            # Camera Joint
            camera_1_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_camera_1_joint", type="fixed")
            camera_1_joint.insert(0, ET.Comment("Camera 1 Joint"))
            camera_1_joint_child = ET.SubElement(camera_1_joint, "child")
            camera_1_joint_child.text = config["model_name"] + "_camera_1_link"
            camera_1_joint_parent = ET.SubElement(camera_1_joint, "parent")
            camera_1_joint_parent.text = config["model_name"] + "_link"


        if ("camera_2" in config and config["camera_2"]):
            # Camera Link
            camera_2_link = ET.SubElement(model, "link", name=config["model_name"] + "_camera_2_link")
            camera_2_link.append(ET.Comment("Camera 1 Link"))
            camera_2_link_pose = ET.SubElement(camera_2_link, "pose", relative_to=config["model_name"] + "_link")
            camera_2_link_pose.text = config["camera_2_pose"]
            camera_2_link_sensor = ET.SubElement(camera_2_link, "sensor", name=config["model_name"] + "_camera_2", type="camera")
            camera_2_link_sensor_camera = ET.SubElement(camera_2_link_sensor, "camera")
            camera_2_link_sensor_camera_image = ET.SubElement(camera_2_link_sensor_camera, "image")
            camera_2_link_sensor_camera_image_width = ET.SubElement(camera_2_link_sensor_camera_image, "width")
            camera_2_link_sensor_camera_image_width.text = str(config["camera_2_width"])
            camera_2_link_sensor_camera_image_height = ET.SubElement(camera_2_link_sensor_camera_image, "height")
            camera_2_link_sensor_camera_image_height.text = str(config["camera_2_height"])
            camera_2_link_sensor_camera_fov = ET.SubElement(camera_2_link_sensor_camera, "horizontal_fov")
            camera_2_link_sensor_camera_fov.text = str(config["camera_2_fov"])
            camera_2_link_sensor_camera_clip = ET.SubElement(camera_2_link_sensor_camera, "clip")
            camera_2_link_sensor_camera_clip_far = ET.SubElement(camera_2_link_sensor_camera_clip, "far")
            camera_2_link_sensor_camera_clip_far.text = str(config["camera_2_clip_far"])
            camera_2_link_sensor_camera_clip_near = ET.SubElement(camera_2_link_sensor_camera_clip, "near")
            camera_2_link_sensor_camera_clip_near.text = str(config["camera_2_clip_near"])
            camera_2_link_sensor_always_on = ET.SubElement(camera_2_link_sensor, "always_on")
            camera_2_link_sensor_always_on.text = "1"
            camera_2_link_sensor_update_rate = ET.SubElement(camera_2_link_sensor, "update_rate")
            camera_2_link_sensor_update_rate.text = str(config["camera_2_update_rate"])
            camera_2_link_sensor_topic = ET.SubElement(camera_2_link_sensor, "topic")
            camera_2_link_sensor_topic.text = config["camera_2_topic"]
            camera_2_link_sensor_camera_info_topic = ET.SubElement(camera_2_link_sensor_camera, "camera_info_topic")
            camera_2_link_sensor_camera_info_topic.text = config["camera_2_info_topic"]
            camera_2_link_inertia = ET.SubElement(camera_2_link, "inertial")
            camera_2_link_inertia_mass = ET.SubElement(camera_2_link_inertia, "mass")
            camera_2_link_inertia_mass.text = "0.0001"
            camera_2_link_sensor_visualize = ET.SubElement(camera_2_link_sensor, "visualize")
            camera_2_link_sensor_visualize.text = "1"

            # Camera Joint
            camera_2_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_camera_2_joint", type="fixed")
            camera_2_joint.insert(0, ET.Comment("Camera 1 Joint"))
            camera_2_joint_child = ET.SubElement(camera_2_joint, "child")
            camera_2_joint_child.text = config["model_name"] + "_camera_2_link"
            camera_2_joint_parent = ET.SubElement(camera_2_joint, "parent")
            camera_2_joint_parent.text = config["model_name"] + "_link"

        if ("camera_3" in config and config["camera_3"]):

            # Camera Link
            camera_3_link = ET.SubElement(model, "link", name=config["model_name"] + "_camera_3_link")
            camera_3_link.append(ET.Comment("Camera 1 Link"))
            camera_3_link_pose = ET.SubElement(camera_3_link, "pose", relative_to=config["model_name"] + "_link")
            camera_3_link_pose.text = config["camera_3_pose"]
            camera_3_link_sensor = ET.SubElement(camera_3_link, "sensor", name=config["model_name"] + "_camera_3", type="camera")
            camera_3_link_sensor_camera = ET.SubElement(camera_3_link_sensor, "camera")
            camera_3_link_sensor_camera_image = ET.SubElement(camera_3_link_sensor_camera, "image")
            camera_3_link_sensor_camera_image_width = ET.SubElement(camera_3_link_sensor_camera_image, "width")
            camera_3_link_sensor_camera_image_width.text = str(config["camera_3_width"])
            camera_3_link_sensor_camera_image_height = ET.SubElement(camera_3_link_sensor_camera_image, "height")
            camera_3_link_sensor_camera_image_height.text = str(config["camera_3_height"])
            camera_3_link_sensor_camera_fov = ET.SubElement(camera_3_link_sensor_camera, "horizontal_fov")
            camera_3_link_sensor_camera_fov.text = str(config["camera_3_fov"])
            camera_3_link_sensor_camera_clip = ET.SubElement(camera_3_link_sensor_camera, "clip")
            camera_3_link_sensor_camera_clip_far = ET.SubElement(camera_3_link_sensor_camera_clip, "far")
            camera_3_link_sensor_camera_clip_far.text = str(config["camera_3_clip_far"])
            camera_3_link_sensor_camera_clip_near = ET.SubElement(camera_3_link_sensor_camera_clip, "near")
            camera_3_link_sensor_camera_clip_near.text = str(config["camera_3_clip_near"])
            camera_3_link_sensor_always_on = ET.SubElement(camera_3_link_sensor, "always_on")
            camera_3_link_sensor_always_on.text = "1"
            camera_3_link_sensor_update_rate = ET.SubElement(camera_3_link_sensor, "update_rate")
            camera_3_link_sensor_update_rate.text = str(config["camera_3_update_rate"])
            camera_3_link_sensor_topic = ET.SubElement(camera_3_link_sensor, "topic")
            camera_3_link_sensor_topic.text = config["camera_3_topic"]
            camera_3_link_sensor_camera_info_topic = ET.SubElement(camera_3_link_sensor_camera, "camera_info_topic")
            camera_3_link_sensor_camera_info_topic.text = config["camera_3_info_topic"]
            camera_3_link_inertia = ET.SubElement(camera_3_link, "inertial")
            camera_3_link_inertia_mass = ET.SubElement(camera_3_link_inertia, "mass")
            camera_3_link_inertia_mass.text = "0.0001"
            camera_3_link_sensor_visualize = ET.SubElement(camera_3_link_sensor, "visualize")
            camera_3_link_sensor_visualize.text = "1"

            # Camera Joint
            camera_3_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_camera_3_joint", type="fixed")
            camera_3_joint.insert(0, ET.Comment("Camera 1 Joint"))
            camera_3_joint_child = ET.SubElement(camera_3_joint, "child")
            camera_3_joint_child.text = config["model_name"] + "_camera_3_link"
            camera_3_joint_parent = ET.SubElement(camera_3_joint, "parent")
            camera_3_joint_parent.text = config["model_name"] + "_link"
        
        if ("depth_camera" in config and config["depth_camera"]):

            # Depth Camera Link
            depth_camera_link = ET.SubElement(model, "link", name=config["model_name"] + "_depth_camera_link")
            depth_camera_link.append(ET.Comment("Depth Camera Link"))
            depth_camera_link_pose = ET.SubElement(depth_camera_link, "pose", relative_to=config["model_name"] + "_link")
            depth_camera_link_pose.text = config["depth_camera_pose"]
            depth_camera_link_sensor = ET.SubElement(depth_camera_link, "sensor", name=config["model_name"] + "_depth_camera", type="depth_camera")
            depth_camera_link_camera = ET.SubElement(depth_camera_link_sensor, "camera")
            depth_camera_link_camera_image = ET.SubElement(depth_camera_link_camera, "image")
            depth_camera_link_camera_image_width = ET.SubElement(depth_camera_link_camera_image, "width")
            depth_camera_link_camera_image_width.text = str(config["depth_camera_width"])
            depth_camera_link_camera_image_height = ET.SubElement(depth_camera_link_camera_image, "height")
            depth_camera_link_camera_image_height.text = str(config["depth_camera_height"])
            depth_camera_link_camera_fov = ET.SubElement(depth_camera_link_camera, "horizontal_fov")
            depth_camera_link_camera_fov.text = str(config["depth_camera_fov"])
            depth_camera_link_camera_clip = ET.SubElement(depth_camera_link_camera, "clip")
            depth_camera_link_camera_clip_far = ET.SubElement(depth_camera_link_camera_clip, "far")
            depth_camera_link_camera_clip_far.text = str(config["depth_camera_clip_far"])
            depth_camera_link_camera_clip_near = ET.SubElement(depth_camera_link_camera_clip, "near")
            depth_camera_link_camera_clip_near.text = str(config["depth_camera_clip_near"])
            depth_camera_link_sensor_always_on = ET.SubElement(depth_camera_link_sensor, "always_on")
            depth_camera_link_sensor_always_on.text = "1"
            depth_camera_link_sensor_update_rate = ET.SubElement(depth_camera_link_sensor, "update_rate")
            depth_camera_link_sensor_update_rate.text = str(config["depth_camera_update_rate"])
            depth_camera_link_sensor_topic = ET.SubElement(depth_camera_link_sensor, "topic")
            depth_camera_link_sensor_topic.text = config["depth_camera_topic"]
            depth_camera_link_sensor_camera_info_topic = ET.SubElement(depth_camera_link_camera, "camera_info_topic")
            depth_camera_link_sensor_camera_info_topic.text = config["depth_camera_info_topic"]
            depth_camera_link_inertia = ET.SubElement(depth_camera_link, "inertial")
            depth_camera_link_inertia_mass = ET.SubElement(depth_camera_link_inertia, "mass")
            depth_camera_link_inertia_mass.text = "0.0001"
            depth_camera_link_depth_camera = ET.SubElement(depth_camera_link_camera, "depth_camera")
            depth_camera_link_depth_camera_output = ET.SubElement(depth_camera_link_depth_camera, "output")
            depth_camera_link_depth_camera_output.text = "/blueye/depth_camera/output"
            depth_camera_link_depth_camera_clip = ET.SubElement(depth_camera_link_depth_camera, "clip")
            depth_camera_link_depth_camera_clip_near = ET.SubElement(depth_camera_link_depth_camera_clip, "near")
            depth_camera_link_depth_camera_clip_near.text = "0.1"
            depth_camera_link_depth_camera_clip_far = ET.SubElement(depth_camera_link_depth_camera_clip, "far")
            depth_camera_link_depth_camera_clip_far.text = "30.0"
            depth_camera_link_sensor_visualize = ET.SubElement(depth_camera_link_sensor, "visualize")
            depth_camera_link_sensor_visualize.text = "1"
            depth_camera_link_camera_intrinsics = ET.SubElement(depth_camera_link_camera, "intrinsics")

            # Depth Camera Joint
            depth_camera_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_depth_camera_joint", type="fixed")
            depth_camera_joint.append(ET.Comment("Depth Camera Joint"))
            depth_camera_joint_child = ET.SubElement(depth_camera_joint, "child")
            depth_camera_joint_child.text = config["model_name"] + "_depth_camera_link"
            depth_camera_joint_parent = ET.SubElement(depth_camera_joint, "parent")
            depth_camera_joint_parent.text = config["model_name"] + "_link"

        if ("dvl" in config and config["dvl"]):
            dvl_ray_link = ET.SubElement(model, "link", name=config["model_name"] + "_dvl_ray_link")
            dvl_ray_link.insert(0, ET.Comment("DVL Ray Link"))
            dvl_ray_link_pose = ET.SubElement(dvl_ray_link, "pose", relative_to=config["model_name"] + "_link")
            dvl_ray_link_pose.text = config["dvl_pose"] + " 0.0 1.5758 0.0"
            dvl_ray_sensor = ET.SubElement(dvl_ray_link, "sensor", name=config["model_name"] + "_dvl_ray", type="gpu_lidar")
            dvl_ray_sensor_always_on = ET.SubElement(dvl_ray_sensor, "always_on")
            dvl_ray_sensor_always_on.text = "1"
            dvl_ray_sensor_update_rate = ET.SubElement(dvl_ray_sensor, "update_rate")
            dvl_ray_sensor_update_rate.text = config["dvl_update_rate"]
            dvl_ray_sensor_topic = ET.SubElement(dvl_ray_sensor, "topic")
            dvl_ray_sensor_topic.text = config["dvl_topic"] + "/ray"
            dvl_ray_sensor_visualize = ET.SubElement(dvl_ray_sensor, "visualize")
            dvl_ray_sensor_visualize.text = "1"
            dvl_ray_sensor_ray = ET.SubElement(dvl_ray_sensor, "ray")
            dvl_ray_sensor_ray_scan = ET.SubElement(dvl_ray_sensor_ray, "scan")
            dvl_ray_sensor_ray_scan_horizontal = ET.SubElement(dvl_ray_sensor_ray_scan, "horizontal")
            dvl_ray_sensor_ray_scan_horizontal_samples = ET.SubElement(dvl_ray_sensor_ray_scan_horizontal, "samples")
            dvl_ray_sensor_ray_scan_horizontal_samples.text = "1"
            dvl_ray_sensor_ray_scan_horizontal_resolution = ET.SubElement(dvl_ray_sensor_ray_scan_horizontal, "resolution")
            dvl_ray_sensor_ray_scan_horizontal_resolution.text = "1"
            dvl_ray_sensor_ray_scan_horizontal_min_angle = ET.SubElement(dvl_ray_sensor_ray_scan_horizontal, "min_angle")
            dvl_ray_sensor_ray_scan_horizontal_min_angle.text = "-0.1"
            dvl_ray_sensor_ray_scan_horizontal_max_angle = ET.SubElement(dvl_ray_sensor_ray_scan_horizontal, "max_angle")
            dvl_ray_sensor_ray_scan_horizontal_max_angle.text = "0.1"
            dvl_ray_sensor_ray_scan_vertical = ET.SubElement(dvl_ray_sensor_ray_scan, "vertical")
            dvl_ray_sensor_ray_scan_vertical_samples = ET.SubElement(dvl_ray_sensor_ray_scan_vertical, "samples")
            dvl_ray_sensor_ray_scan_vertical_samples.text = "1"
            dvl_ray_sensor_ray_scan_vertical_resolution = ET.SubElement(dvl_ray_sensor_ray_scan_vertical, "resolution")
            dvl_ray_sensor_ray_scan_vertical_resolution.text = "1"
            dvl_ray_sensor_ray_scan_vertical_min_angle = ET.SubElement(dvl_ray_sensor_ray_scan_vertical, "min_angle")
            dvl_ray_sensor_ray_scan_vertical_min_angle.text = "-0.1"
            dvl_ray_sensor_ray_scan_vertical_max_angle = ET.SubElement(dvl_ray_sensor_ray_scan_vertical, "max_angle")
            dvl_ray_sensor_ray_scan_vertical_max_angle.text = "0.1"
            dvl_ray_sensor_ray_range = ET.SubElement(dvl_ray_sensor_ray, "range")
            dvl_ray_sensor_ray_range_min = ET.SubElement(dvl_ray_sensor_ray_range, "min")
            dvl_ray_sensor_ray_range_min.text = "0.1"
            dvl_ray_sensor_ray_range_max = ET.SubElement(dvl_ray_sensor_ray_range, "max")
            dvl_ray_sensor_ray_range_max.text = "30.0"
            dvl_ray_sensor_ray_range_resolution = ET.SubElement(dvl_ray_sensor_ray_range, "resolution")
            dvl_ray_sensor_ray_range_resolution.text = "0.1"
            dvl_ray_sensor_ray_noise = ET.SubElement(dvl_ray_sensor_ray, "noise")
            dvl_ray_sensor_ray_noise_type = ET.SubElement(dvl_ray_sensor_ray_noise, "type")
            dvl_ray_sensor_ray_noise_type.text = "gaussian"
            dvl_ray_sensor_ray_noise_mean = ET.SubElement(dvl_ray_sensor_ray_noise, "mean")
            dvl_ray_sensor_ray_noise_mean.text = str(config["dvl_range_noise_mean"])
            dvl_ray_sensor_ray_noise_stddev = ET.SubElement(dvl_ray_sensor_ray_noise, "stddev")
            dvl_ray_sensor_ray_noise_stddev.text = str(config["dvl_range_noise_stddev"])
            dvl_ray_sensor_link_inertia = ET.SubElement(dvl_ray_link, "inertial")
            dvl_ray_sensor_link_inertia_mass = ET.SubElement(dvl_ray_sensor_link_inertia, "mass")
            dvl_ray_sensor_link_inertia_mass.text = "0.0001"

            # Joint
            dvl_ray_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_dvl_ray_joint", type="fixed")
            dvl_ray_joint.insert(0, ET.Comment("DVL Ray Joint"))
            dvl_ray_joint_child = ET.SubElement(dvl_ray_joint, "child")
            dvl_ray_joint_child.text = config["model_name"] + "_dvl_ray_link"
            dvl_ray_joint_parent = ET.SubElement(dvl_ray_joint, "parent")
            dvl_ray_joint_parent.text = config["model_name"] + "_link"

        if ("sonar" in config and config["sonar"]):
            # Link
            sonar_link = ET.SubElement(model, "link", name=config["model_name"] + "_sonar_link")
            sonar_link.insert(0, ET.Comment("Sonar Link"))
            sonar_link_pose = ET.SubElement(sonar_link, "pose", relative_to=config["model_name"] + "_link")
            sonar_link_pose.text = config["sonar_pose"]
            sonar_sensor = ET.SubElement(sonar_link, "sensor", name=config["model_name"] + "_sonar", type="gpu_lidar")
            #sonar_sensor_ignition_frame = ET.SubElement(sonar_sensor, "ignition_frame")
            #sonar_sensor_ignition_frame.text = "sonar_link"
            sonar_sensor_always_on = ET.SubElement(sonar_sensor, "always_on")
            sonar_sensor_always_on.text = "1"
            sonar_sensor_update_rate = ET.SubElement(sonar_sensor, "update_rate")
            sonar_sensor_update_rate.text = config["sonar_update_rate"]
            sonar_sensor_topic = ET.SubElement(sonar_sensor, "topic")
            sonar_sensor_topic.text = config["sonar_topic"]
            sonar_sensor_visualize = ET.SubElement(sonar_sensor, "visualize")
            sonar_sensor_visualize.text = "1"
            sonar_sensor_ray = ET.SubElement(sonar_sensor, "ray")
            sonar_sensor_ray_scan = ET.SubElement(sonar_sensor_ray, "scan")
            sonar_sensor_ray_scan_horizontal = ET.SubElement(sonar_sensor_ray_scan, "horizontal")
            sonar_sensor_ray_scan_horizontal_samples = ET.SubElement(sonar_sensor_ray_scan_horizontal, "samples")
            sonar_sensor_ray_scan_horizontal_samples.text = str(config["sonar_horizontal_samples"])
            sonar_sensor_ray_scan_horizontal_resolution = ET.SubElement(sonar_sensor_ray_scan_horizontal, "resolution")
            sonar_sensor_ray_scan_horizontal_resolution.text = str(config["sonar_horizontal_resolution"])
            sonar_sensor_ray_scan_horizontal_min_angle = ET.SubElement(sonar_sensor_ray_scan_horizontal, "min_angle")
            sonar_sensor_ray_scan_horizontal_min_angle.text = str(config["sonar_horizontal_min_angle"])
            sonar_sensor_ray_scan_horizontal_max_angle = ET.SubElement(sonar_sensor_ray_scan_horizontal, "max_angle")
            sonar_sensor_ray_scan_horizontal_max_angle.text = str(config["sonar_horizontal_max_angle"])
            sonar_sensor_ray_scan_vertical = ET.SubElement(sonar_sensor_ray_scan, "vertical")
            sonar_sensor_ray_scan_vertical_samples = ET.SubElement(sonar_sensor_ray_scan_vertical, "samples")
            sonar_sensor_ray_scan_vertical_samples.text = str(config["sonar_vertical_samples"])
            sonar_sensor_ray_scan_vertical_resolution = ET.SubElement(sonar_sensor_ray_scan_vertical, "resolution")
            sonar_sensor_ray_scan_vertical_resolution.text = str(config["sonar_vertical_resolution"])
            sonar_sensor_ray_scan_vertical_min_angle = ET.SubElement(sonar_sensor_ray_scan_vertical, "min_angle")
            sonar_sensor_ray_scan_vertical_min_angle.text = str(config["sonar_vertical_min_angle"])
            sonar_sensor_ray_scan_vertical_max_angle = ET.SubElement(sonar_sensor_ray_scan_vertical, "max_angle")
            sonar_sensor_ray_scan_vertical_max_angle.text = str(config["sonar_vertical_max_angle"])
            sonar_sensor_ray_range = ET.SubElement(sonar_sensor_ray, "range")
            sonar_sensor_ray_range_min = ET.SubElement(sonar_sensor_ray_range, "min")
            sonar_sensor_ray_range_min.text = str(config["sonar_range_min"])
            sonar_sensor_ray_range_max = ET.SubElement(sonar_sensor_ray_range, "max")
            sonar_sensor_ray_range_max.text = str(config["sonar_range_max"])
            sonar_sensor_ray_range_resolution = ET.SubElement(sonar_sensor_ray_range, "resolution")
            sonar_sensor_ray_range_resolution.text = str(config["sonar_range_resolution"])
            sonar_sensor_ray_noise = ET.SubElement(sonar_sensor_ray, "noise")
            sonar_sensor_ray_noise_type = ET.SubElement(sonar_sensor_ray_noise, "type")
            sonar_sensor_ray_noise_type.text = "gaussian"
            sonar_sensor_ray_noise_mean = ET.SubElement(sonar_sensor_ray_noise, "mean")
            sonar_sensor_ray_noise_mean.text = str(config["sonar_noise_mean"])
            sonar_sensor_ray_noise_stddev = ET.SubElement(sonar_sensor_ray_noise, "stddev")
            sonar_sensor_ray_noise_stddev.text = str(config["sonar_noise_stddev"])
            sonar_sensor_link_inertia = ET.SubElement(sonar_link, "inertial")
            sonar_sensor_link_inertia_mass = ET.SubElement(sonar_sensor_link_inertia, "mass")
            sonar_sensor_link_inertia_mass.text = "0.0001"

            # Joint
            sonar_joint = ET.SubElement(model, "joint", name=config["model_name"] + "_sonar_joint", type="fixed")
            sonar_joint.insert(0, ET.Comment("Sonar Joint"))
            sonar_joint_child = ET.SubElement(sonar_joint, "child")
            sonar_joint_child.text = config["model_name"] + "_sonar_link"
            sonar_joint_parent = ET.SubElement(sonar_joint, "parent")
            sonar_joint_parent.text = config["model_name"] + "_link"

        # Actuators
        if ("thruster_1" in config and config["thruster_1"]):    
            thruster_1 = ET.SubElement(model, "link", name=config["model_name"] + "_thruster_1_link")
            thruster_1.insert(0, ET.Comment("Thruster 1 Link"))
            thruster_1_pose = ET.SubElement(thruster_1, "pose", relative_to=config["model_name"] + "_link")
            thruster_1_pose.text = config["thruster_1_pose"]
            thruster_1_visual = ET.SubElement(thruster_1, "visual", name=config["model_name"] + "_thruster_1_visual")
            thruster_1_visual_pose = ET.SubElement(thruster_1_visual, "pose", relative_to=config["model_name"] + "_link")
            thruster_1_visual_pose.text = "-0.14 0.0815 -0.004 0.0 -3.1415 0.0"
            thruster_1_visual_geometry = ET.SubElement(thruster_1_visual, "geometry")
            thruster_1_visual_geometry_mesh = ET.SubElement(thruster_1_visual_geometry, "mesh")
            thruster_1_visual_geometry_mesh_uri = ET.SubElement(thruster_1_visual_geometry_mesh, "uri")
            thruster_1_visual_geometry_mesh_uri.text = "model://models/dynamic_assets/" + config["model_name"] + "/meshes/prop.dae"
            thruster_1_visual_geometry_mesh_scale = ET.SubElement(thruster_1_visual_geometry_mesh, "scale")
            thruster_1_visual_geometry_mesh_scale.text = "0.7 0.7 0.7"
            thruster_1_inertia = ET.SubElement(thruster_1, "inertial")
            thruster_1_inertia_mass = ET.SubElement(thruster_1_inertia, "mass")
            thruster_1_inertia_mass.text = "0.0001"

            thruster_1_joint  = ET.SubElement(model, "joint", name=config["model_name"] + "_thruster_1_joint", type="revolute")
            thruster_1_joint.insert(0, ET.Comment("Thruster 1 Joint"))
            thruster_1_joint_child = ET.SubElement(thruster_1_joint, "child")
            thruster_1_joint_child.text = config["model_name"] + "_thruster_1_link"
            thruster_1_joint_parent = ET.SubElement(thruster_1_joint, "parent")
            thruster_1_joint_parent.text = config["model_name"] + "_link"
            thruster_1_joint_axis = ET.SubElement(thruster_1_joint, "axis")
            thruster_1_joint_axis_xyz = ET.SubElement(thruster_1_joint_axis, "xyz")
            thruster_1_joint_axis_xyz.text = "0 0 -1"
            thruster_1_joint_axis_limit = ET.SubElement(thruster_1_joint_axis, "limit")
            thruster_1_joint_axis_limit_lower = ET.SubElement(thruster_1_joint_axis_limit, "lower")
            thruster_1_joint_axis_limit_lower.text = "-1e+16"
            thruster_1_joint_axis_limit_upper = ET.SubElement(thruster_1_joint_axis_limit, "upper")
            thruster_1_joint_axis_limit_upper.text = "1e+16"

            thruster_1_plugin = ET.SubElement(model, "plugin", name="gz::sim::systems::Thruster", filename="gz-sim-thruster-system")
            thruster_1_plugin_namespace = ET.SubElement(thruster_1_plugin, "namespace")
            thruster_1_plugin_namespace.text = config["model_name"]
            thruster_1_joint_name = ET.SubElement(thruster_1_plugin, "joint_name")
            thruster_1_joint_name.text = config["model_name"] + "_thruster_1_joint"
            thruster_1_fluid_density = ET.SubElement(thruster_1_plugin, "fluid_density")
            thruster_1_fluid_density.text = "1025"
            thruster_1_propeller_diameter = ET.SubElement(thruster_1_plugin, "propeller_diameter")
            thruster_1_propeller_diameter.text = config["thruster_1_diameter"]
            thruster_1_max_thrust_cmd = ET.SubElement(thruster_1_plugin, "max_thrust_cmd")
            thruster_1_max_thrust_cmd.text = config["thruster_1_max_thrust"]
            thruster_1_min_thrust_cmd = ET.SubElement(thruster_1_plugin, "min_thrust_cmd")
            thruster_1_min_thrust_cmd.text = config["thruster_1_min_thrust"]


        if ("thruster_2" in config and config["thruster_2"]):    
            thruster_2 = ET.SubElement(model, "link", name=config["model_name"] + "_thruster_2_link")
            thruster_2.insert(0, ET.Comment("Thruster 1 Link"))
            thruster_2_pose = ET.SubElement(thruster_2, "pose", relative_to=config["model_name"] + "_link")
            thruster_2_pose.text = config["thruster_2_pose"]
            thruster_2_visual = ET.SubElement(thruster_2, "visual", name=config["model_name"] + "_thruster_2_visual")
            thruster_2_visual_pose = ET.SubElement(thruster_2_visual, "pose", relative_to=config["model_name"] + "_link")
            thruster_2_visual_pose.text = "-0.14 -0.0785 -0.004 0.0 -3.1415 0.0"
            thruster_2_visual_geometry = ET.SubElement(thruster_2_visual, "geometry")
            thruster_2_visual_geometry_mesh = ET.SubElement(thruster_2_visual_geometry, "mesh")
            thruster_2_visual_geometry_mesh_uri = ET.SubElement(thruster_2_visual_geometry_mesh, "uri")
            thruster_2_visual_geometry_mesh_uri.text = "model://models/dynamic_assets/" + config["model_name"] + "/meshes/prop.dae"
            thruster_2_visual_geometry_mesh_scale = ET.SubElement(thruster_2_visual_geometry_mesh, "scale")
            thruster_2_visual_geometry_mesh_scale.text = "0.7 0.7 0.7"
            thruster_2_inertia = ET.SubElement(thruster_2, "inertial")
            thruster_2_inertia_mass = ET.SubElement(thruster_2_inertia, "mass")
            thruster_2_inertia_mass.text = "0.0001"

            thruster_2_joint  = ET.SubElement(model, "joint", name=config["model_name"] + "_thruster_2_joint", type="revolute")
            thruster_2_joint.insert(0, ET.Comment("Thruster 1 Joint"))
            thruster_2_joint_child = ET.SubElement(thruster_2_joint, "child")
            thruster_2_joint_child.text = config["model_name"] + "_thruster_2_link"
            thruster_2_joint_parent = ET.SubElement(thruster_2_joint, "parent")
            thruster_2_joint_parent.text = config["model_name"] + "_link"
            thruster_2_joint_axis = ET.SubElement(thruster_2_joint, "axis")
            thruster_2_joint_axis_xyz = ET.SubElement(thruster_2_joint_axis, "xyz")
            thruster_2_joint_axis_xyz.text = "0 0 -1"
            thruster_2_joint_axis_limit = ET.SubElement(thruster_2_joint_axis, "limit")
            thruster_2_joint_axis_limit_lower = ET.SubElement(thruster_2_joint_axis_limit, "lower")
            thruster_2_joint_axis_limit_lower.text = "-1e+16"
            thruster_2_joint_axis_limit_upper = ET.SubElement(thruster_2_joint_axis_limit, "upper")
            thruster_2_joint_axis_limit_upper.text = "1e+16"

            thruster_2_plugin = ET.SubElement(model, "plugin", name="gz::sim::systems::Thruster", filename="gz-sim-thruster-system")
            thruster_2_plugin_namespace = ET.SubElement(thruster_2_plugin, "namespace")
            thruster_2_plugin_namespace.text = config["model_name"]
            thruster_2_joint_name = ET.SubElement(thruster_2_plugin, "joint_name")
            thruster_2_joint_name.text = config["model_name"] + "_thruster_2_joint"
            thruster_2_fluid_density = ET.SubElement(thruster_2_plugin, "fluid_density")
            thruster_2_fluid_density.text = "1025"
            thruster_2_propeller_diameter = ET.SubElement(thruster_2_plugin, "propeller_diameter")
            thruster_2_propeller_diameter.text = config["thruster_2_diameter"]
            thruster_2_max_thrust_cmd = ET.SubElement(thruster_2_plugin, "max_thrust_cmd")
            thruster_2_max_thrust_cmd.text = config["thruster_2_max_thrust"]
            thruster_2_min_thrust_cmd = ET.SubElement(thruster_2_plugin, "min_thrust_cmd")
            thruster_2_min_thrust_cmd.text = config["thruster_2_min_thrust"]

        if ("thruster_3" in config and config["thruster_3"]):    
            thruster_3 = ET.SubElement(model, "link", name=config["model_name"] + "_thruster_3_link")
            thruster_3.insert(0, ET.Comment("Thruster 1 Link"))
            thruster_3_pose = ET.SubElement(thruster_3, "pose", relative_to=config["model_name"] + "_link")
            thruster_3_pose.text = config["thruster_3_pose"]
            thruster_3_visual = ET.SubElement(thruster_3, "visual", name=config["model_name"] + "_thruster_3_visual")
            thruster_3_visual_pose = ET.SubElement(thruster_3_visual, "pose")
            thruster_3_visual_pose.text = "0.0 0.0 0.0 0.0 -1.5707 0.0"
            thruster_3_visual_geometry = ET.SubElement(thruster_3_visual, "geometry")
            thruster_3_visual_geometry_mesh = ET.SubElement(thruster_3_visual_geometry, "mesh")
            thruster_3_visual_geometry_mesh_uri = ET.SubElement(thruster_3_visual_geometry_mesh, "uri")
            thruster_3_visual_geometry_mesh_uri.text = "model://models/dynamic_assets/" + config["model_name"] + "/meshes/prop.dae"
            thruster_3_visual_geometry_mesh_scale = ET.SubElement(thruster_3_visual_geometry_mesh, "scale")
            thruster_3_visual_geometry_mesh_scale.text = "0.7 0.7 0.7"
            thruster_3_inertia = ET.SubElement(thruster_3, "inertial")
            thruster_3_inertia_mass = ET.SubElement(thruster_3_inertia, "mass")
            thruster_3_inertia_mass.text = "0.0001"

            thruster_3_joint  = ET.SubElement(model, "joint", name=config["model_name"] + "_thruster_3_joint", type="revolute")
            thruster_3_joint.insert(0, ET.Comment("Thruster 1 Joint"))
            thruster_3_joint_child = ET.SubElement(thruster_3_joint, "child")
            thruster_3_joint_child.text = config["model_name"] + "_thruster_3_link"
            thruster_3_joint_parent = ET.SubElement(thruster_3_joint, "parent")
            thruster_3_joint_parent.text = config["model_name"] + "_link"
            thruster_3_joint_axis = ET.SubElement(thruster_3_joint, "axis")
            thruster_3_joint_axis_xyz = ET.SubElement(thruster_3_joint_axis, "xyz")
            thruster_3_joint_axis_xyz.text = "0 0 -1"
            thruster_3_joint_axis_limit = ET.SubElement(thruster_3_joint_axis, "limit")
            thruster_3_joint_axis_limit_lower = ET.SubElement(thruster_3_joint_axis_limit, "lower")
            thruster_3_joint_axis_limit_lower.text = "-1e+16"
            thruster_3_joint_axis_limit_upper = ET.SubElement(thruster_3_joint_axis_limit, "upper")
            thruster_3_joint_axis_limit_upper.text = "1e+16"

            thruster_3_plugin = ET.SubElement(model, "plugin", name="gz::sim::systems::Thruster", filename="gz-sim-thruster-system")
            thruster_3_plugin_namespace = ET.SubElement(thruster_3_plugin, "namespace")
            thruster_3_plugin_namespace.text = config["model_name"]
            thruster_3_joint_name = ET.SubElement(thruster_3_plugin, "joint_name")
            thruster_3_joint_name.text = config["model_name"] + "_thruster_3_joint"
            thruster_3_fluid_density = ET.SubElement(thruster_3_plugin, "fluid_density")
            thruster_3_fluid_density.text = "1025"
            thruster_3_propeller_diameter = ET.SubElement(thruster_3_plugin, "propeller_diameter")
            thruster_3_propeller_diameter.text = config["thruster_3_diameter"]
            thruster_3_max_thrust_cmd = ET.SubElement(thruster_3_plugin, "max_thrust_cmd")
            thruster_3_max_thrust_cmd.text = config["thruster_3_max_thrust"]
            thruster_3_min_thrust_cmd = ET.SubElement(thruster_3_plugin, "min_thrust_cmd")
            thruster_3_min_thrust_cmd.text = config["thruster_3_min_thrust"]

        if ("thruster_4" in config and config["thruster_4"]):    
            thruster_4 = ET.SubElement(model, "link", name=config["model_name"] + "_thruster_4_link")
            thruster_4.insert(0, ET.Comment("Thruster 1 Link"))
            thruster_4_pose = ET.SubElement(thruster_4, "pose", relative_to=config["model_name"] + "_link")
            thruster_4_pose.text = config["thruster_4_pose"]
            thruster_4_visual = ET.SubElement(thruster_4, "visual", name=config["model_name"] + "_thruster_4_visual")
            thruster_4_visual_pose = ET.SubElement(thruster_4_visual, "pose")
            thruster_4_visual_pose.text = "0.0 0.0 0.0 0.0 -1.5707 0.0"
            thruster_4_visual_geometry = ET.SubElement(thruster_4_visual, "geometry")
            thruster_4_visual_geometry_mesh = ET.SubElement(thruster_4_visual_geometry, "mesh")
            thruster_4_visual_geometry_mesh_uri = ET.SubElement(thruster_4_visual_geometry_mesh, "uri")
            thruster_4_visual_geometry_mesh_uri.text = "model://models/dynamic_assets/" + config["model_name"] + "/meshes/prop.dae"
            thruster_4_visual_geometry_mesh_scale = ET.SubElement(thruster_4_visual_geometry_mesh, "scale")
            thruster_4_visual_geometry_mesh_scale.text = "0.7 0.7 0.7"
            thruster_4_inertia = ET.SubElement(thruster_4, "inertial")
            thruster_4_inertia_mass = ET.SubElement(thruster_4_inertia, "mass")
            thruster_4_inertia_mass.text = "0.0001"

            thruster_4_joint  = ET.SubElement(model, "joint", name=config["model_name"] + "_thruster_4_joint", type="revolute")
            thruster_4_joint.insert(0, ET.Comment("Thruster 1 Joint"))
            thruster_4_joint_child = ET.SubElement(thruster_4_joint, "child")
            thruster_4_joint_child.text = config["model_name"] + "_thruster_4_link"
            thruster_4_joint_parent = ET.SubElement(thruster_4_joint, "parent")
            thruster_4_joint_parent.text = config["model_name"] + "_link"
            thruster_4_joint_axis = ET.SubElement(thruster_4_joint, "axis")
            thruster_4_joint_axis_xyz = ET.SubElement(thruster_4_joint_axis, "xyz")
            thruster_4_joint_axis_xyz.text = "0 0 -1"
            thruster_4_joint_axis_limit = ET.SubElement(thruster_4_joint_axis, "limit")
            thruster_4_joint_axis_limit_lower = ET.SubElement(thruster_4_joint_axis_limit, "lower")
            thruster_4_joint_axis_limit_lower.text = "-1e+16"
            thruster_4_joint_axis_limit_upper = ET.SubElement(thruster_4_joint_axis_limit, "upper")
            thruster_4_joint_axis_limit_upper.text = "1e+16"

            thruster_4_plugin = ET.SubElement(model, "plugin", name="gz::sim::systems::Thruster", filename="gz-sim-thruster-system")
            thruster_4_plugin_namespace = ET.SubElement(thruster_4_plugin, "namespace")
            thruster_4_plugin_namespace.text = config["model_name"]
            thruster_4_joint_name = ET.SubElement(thruster_4_plugin, "joint_name")
            thruster_4_joint_name.text = config["model_name"] + "_thruster_4_joint"
            thruster_4_fluid_density = ET.SubElement(thruster_4_plugin, "fluid_density")
            thruster_4_fluid_density.text = "1025"
            thruster_4_propeller_diameter = ET.SubElement(thruster_4_plugin, "propeller_diameter")
            thruster_4_propeller_diameter.text = config["thruster_4_diameter"]
            thruster_4_max_thrust_cmd = ET.SubElement(thruster_4_plugin, "max_thrust_cmd")
            thruster_4_max_thrust_cmd.text = config["thruster_4_max_thrust"]
            thruster_4_min_thrust_cmd = ET.SubElement(thruster_4_plugin, "min_thrust_cmd")
            thruster_4_min_thrust_cmd.text = config["thruster_4_min_thrust"]

        # HydroDynamic Plugin - Added mass are moved to link to make it "proper"
        hydrodynamics = ET.SubElement(model, "plugin", name="gz::sim::systems::Hydrodynamics", filename="gz-sim-hydrodynamics-system")
        hydrodynamics.insert(0, ET.Comment("Hydrodynamic Plugin"))
        hydrodynamic_link_name = ET.SubElement(hydrodynamics, "link_name")
        hydrodynamic_link_name.text = config["model_name"] + "_link"
        hydrodynamic_link_name.append(ET.Comment("Linear Damping"))
        hydrodynamic_xU = ET.SubElement(hydrodynamics, "xU")
        hydrodynamic_xU.text = config["xU"]
        hydrodynamic_yV = ET.SubElement(hydrodynamics, "yV")
        hydrodynamic_yV.text = config["yV"]
        hydrodynamic_zW = ET.SubElement(hydrodynamics, "zW")
        hydrodynamic_zW.text = config["zW"]
        hydrodynamic_kP = ET.SubElement(hydrodynamics, "kP")
        hydrodynamic_kP.text = config["kP"]
        hydrodynamic_mQ = ET.SubElement(hydrodynamics, "mQ")
        hydrodynamic_mQ.text = config["mQ"]
        hydrodynamic_nR = ET.SubElement(hydrodynamics, "nR")
        hydrodynamic_nR.text = config["nR"]
        hydrodynamic_nR.append(ET.Comment("Quadratic Damping"))
        hydrodynamic_xUU = ET.SubElement(hydrodynamics, "xUabsU")
        hydrodynamic_xUU.text = config["xUabsU"]
        hydrodynamic_yVV = ET.SubElement(hydrodynamics, "yVabsV")
        hydrodynamic_yVV.text = config["yVabsV"]
        hydrodynamic_zWW = ET.SubElement(hydrodynamics, "zWabsW")
        hydrodynamic_zWW.text = config["zWabsW"]
        hydrodynamic_kPP = ET.SubElement(hydrodynamics, "kPabsP")
        hydrodynamic_kPP.text = config["kPabsP"]
        hydrodynamic_mQQ = ET.SubElement(hydrodynamics, "mQabsQ")
        hydrodynamic_mQQ.text = config["mQabsQ"]
        hydrodynamic_nRR = ET.SubElement(hydrodynamics, "nRabsR")
        hydrodynamic_nRR.text = config["nRabsR"]

        share_dir = get_package_share_directory("gz_models")
        save_dir = os.path.abspath(os.path.join(share_dir, '../../../../src/mundus_mir_simulator/mundus_mir_gz/gz_models/models/dynamic_assets/', config["model_name"], "generated.sdf"))
        sdf_tree = prettify(root)
        with open(save_dir, "w") as f:
            f.write(sdf_tree)

        
        # Generate ROS2 bridge file based on the parameters
        bridge_config = []

        # We always want odometry
        bridge_config.append({
            'ros_topic_name': config["model_name"] + "/odometry_flu/gt",
            'gz_topic_name': config["model_name"] + "/odometry_flu/gt",
            'ros_type_name': "nav_msgs/msg/Odometry",
            'gz_type_name': "gz.msgs.Odometry",
            'direction': "GZ_TO_ROS"
        })


        if ("thruster_1" in config and config['thruster_1']):
            bridge_config.append({
                'ros_topic_name': config["model_name"] + "/thruster_1/cmd_vel",
                'gz_topic_name': "/model/" + config["model_name"] + "/joint/" + config["model_name"] + "_thruster_1_joint/cmd_thrust",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })

        if ("thruster_2" in config and config['thruster_2']):
            bridge_config.append({
                'ros_topic_name': config["model_name"] + "/thruster_2/cmd_vel",
                'gz_topic_name': "/model/" + config["model_name"] + "/joint/" + config["model_name"] + "_thruster_2_joint/cmd_thrust",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })

        if ("thruster_3" in config and config['thruster_3']):
            bridge_config.append({
                'ros_topic_name': config["model_name"] + "/thruster_3/cmd_vel",
                'gz_topic_name': "/model/" + config["model_name"] + "/joint/" + config["model_name"] + "_thruster_3_joint/cmd_thrust",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })

        if ("thruster_4" in config and config['thruster_4']):
            bridge_config.append({
                'ros_topic_name': config["model_name"] + "/thruster_4/cmd_vel",
                'gz_topic_name': "/model/" + config["model_name"] + "/joint/" + config["model_name"] + "_thruster_4_joint/cmd_thrust",
                'ros_type_name': "std_msgs/msg/Float64",
                'gz_type_name': "gz.msgs.Double",
                'direction': "ROS_TO_GZ"
            })


        if ("imu" in config and config['imu']):
            bridge_config.append({
                'ros_topic_name': config["imu_topic"],
                'gz_topic_name': config["imu_topic"],
                'ros_type_name': "sensor_msgs/msg/Imu",
                'gz_type_name': "gz.msgs.IMU",
                'direction': "GZ_TO_ROS"
            })

        if ("camera_1" in config and config['camera_1']):
            bridge_config.append({
                'ros_topic_name': config["camera_1_topic"],
                'gz_topic_name': config["camera_1_topic"],
                'ros_type_name': "sensor_msgs/msg/Image",
                'gz_type_name': "gz.msgs.Image",
                'direction': "GZ_TO_ROS"
            })

            bridge_config.append({
                'ros_topic_name': config["camera_1_info_topic"],
                'gz_topic_name': config["camera_1_info_topic"],
                'ros_type_name': "sensor_msgs/msg/CameraInfo",
                'gz_type_name': "gz.msgs.CameraInfo",
                'direction': "GZ_TO_ROS"
            })

        if ("camera_2" in config and config['camera_2']):
            bridge_config.append({
                'ros_topic_name': config["camera_2_topic"],
                'gz_topic_name': config["camera_2_topic"],
                'ros_type_name': "sensor_msgs/msg/Image",
                'gz_type_name': "gz.msgs.Image",
                'direction': "GZ_TO_ROS"
            })

            bridge_config.append({
                'ros_topic_name': config["camera_2_info_topic"],
                'gz_topic_name': config["camera_2_info_topic"],
                'ros_type_name': "sensor_msgs/msg/CameraInfo",
                'gz_type_name': "gz.msgs.CameraInfo",
                'direction': "GZ_TO_ROS"
            })

        if ("camera_3" in config and config['camera_3']):
            bridge_config.append({
                'ros_topic_name': config["camera_3_topic"],
                'gz_topic_name': config["camera_3_topic"],
                'ros_type_name': "sensor_msgs/msg/Image",
                'gz_type_name': "gz.msgs.Image",
                'direction': "GZ_TO_ROS"
            })

            bridge_config.append({
                'ros_topic_name': config["camera_3_info_topic"],
                'gz_topic_name': config["camera_3_info_topic"],
                'ros_type_name': "sensor_msgs/msg/CameraInfo",
                'gz_type_name': "gz.msgs.CameraInfo",
                'direction': "GZ_TO_ROS"
            })

        if ("sonar" in config and config['sonar']):
            bridge_config.append({
                'ros_topic_name': config["sonar_topic"],
                'gz_topic_name': config["sonar_topic"],
                'ros_type_name': "sensor_msgs/msg/LaserScan",
                'gz_type_name': "gz.msgs.LaserScan",
                'direction': "GZ_TO_ROS"
            })

        if ("dvl" in config and config["dvl"]):
            bridge_config.append({
                'ros_topic_name': config["dvl_topic"] + "/ray",
                'gz_topic_name': config["dvl_topic"] + "/ray",
                'ros_type_name': "sensor_msgs/msg/LaserScan",
                'gz_type_name': "gz.msgs.LaserScan",
                'direction': "GZ_TO_ROS"
            })
        
        if ("depth_camera" in config and config["depth_camera"]):
            bridge_config.append({
                'ros_topic_name': config["depth_camera_topic"],
                'gz_topic_name': config["depth_camera_topic"],
                'ros_type_name': "sensor_msgs/msg/Image",
                'gz_type_name': "gz.msgs.Image",
                'direction': "GZ_TO_ROS"
            })

            bridge_config.append({
                'ros_topic_name': config["depth_camera_info_topic"],
                'gz_topic_name': config["depth_camera_info_topic"],
                'ros_type_name': "sensor_msgs/msg/CameraInfo",
                'gz_type_name': "gz.msgs.CameraInfo",
                'direction': "GZ_TO_ROS"
            })


        share_launch_dir = get_package_share_directory("mundus_mir_simulator_launch")
        # Temporarly, how to make this path relative to the launchfile?
        save_path = os.path.join(share_launch_dir, "config", self.launch_file, "generated_bridge.yaml")
        with open(save_path, 'w') as file:
            yaml.dump(bridge_config, file, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)
    sdf_generator = SDF_Generator()
    sdf_generator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()