import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition

def generate_launch_description():
    # Get the launch directory
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    pb_rm_simulation_launch_dir = os.path.join(get_package_share_directory('pb_rm_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'config', 'simulation', 'measurement_params_sim.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_nav_bringup'), 'urdf', 'sentry_robot_sim.xacro'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    segmentation_params = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'segmentation_sim.yaml')
    ########################## linefit_ground_segementation parameters end ############################

    #################################### FAST_LIO parameters start ####################################
    fastlio_mid360_params = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'fastlio.rviz')
    ##################################### FAST_LIO parameters end #####################################

    ################################### POINT_LIO parameters start ####################################
    pointlio_mid360_params = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'pointlio_mid360_sim.yaml')
    pointlio_rviz_cfg_dir = os.path.join(rm_nav_bringup_dir, 'rviz', 'pointlio.rviz')
    #################################### POINT_LIO parameters end #####################################

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world])
    slam_toolbox_localization_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'mapper_params_localization_sim.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'mapper_params_online_async_sim.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    # 仿真专用地图：固定使用 RMUL（非 11_map/Desktop 等实机地图）
    nav2_map_path = os.path.join(rm_nav_bringup_dir, 'map', 'RMUL.yaml')
    nav2_params_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'nav2_params_sim.yaml')
    ################################### navigation2 parameters end ####################################

    ################################ icp_registration parameters start ################################
    icp_pcd_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'PCD', world]), ".pcd"
    icp_registration_params_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'icp_registration_sim.yaml')
    ################################# icp_registration parameters end #################################

    ############################# pointcloud_downsampling parameters start ############################
    pointcloud_downsampling_config_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'pointcloud_downsampling_sim.yaml')
    ############################# pointcloud_downsampling parameters start ############################

    # Declare launch options
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')

    # 仿真固定用 RMUL，不暴露 map 参数避免误传 11_map
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fast_lio',
        description='Choose lio alogrithm: fastlio or pointlio')

    # Specify the actions
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo without GUI (much lighter)'
    )

    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pb_rm_simulation_launch_dir, 'rm_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
            'rviz': 'False',
            'headless': LaunchConfiguration('headless')}.items()
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )

    # 仿真专用：始终订阅点云并发布 /scan，无懒订阅
    pc2scan_script = os.path.join(
        get_package_share_directory('rm_nav_bringup'), 'scripts', 'pointcloud_to_laserscan_sim.py')
    bringup_pointcloud_to_laserscan_node = ExecuteProcess(
        cmd=['python3', pc2scan_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        emulate_tty=True,
    )

    bringup_LIO_group = GroupAction([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'lidar_odom'
            ],
        ),
        # 连接 Fast-LIO 与 Nav2：lidar_odom<->camera_init, body<->base_link，使 map 与 base_link_fake 在同一 TF 树
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_odom_to_camera_init",
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_odom', 'camera_init'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # dummy 是 URDF 根，body 是 Fast-LIO 的机器人帧，连接后：map->odom->lidar_odom->camera_init->body->dummy->base_link->base_link_fake
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_dummy",
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'dummy'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    {use_sim_time: use_sim_time}
                ],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            ),
        ]),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'pointlio'),
            actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                output='screen',
                parameters=[
                    pointlio_mid360_params,
                    {'use_sim_time': use_sim_time,
                    'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                    'prop_at_freq_of_imu': True,
                    'check_satu': False,
                    'init_map_size': 10,
                    'point_filter_num': 3,  # Options: 1, 3
                    'space_down_sample': True,
                    'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                    'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                    'ivox_nearby_type': 26,   # Options: 0, 6, 18, 26
                    'runtime_pos_log_enable': False}
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
            )
        ])
    ])

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            # 保底：立即发布 map->odom，确保 map 坐标系存在，避免 RViz/Nav2 报 "frame does not exist"
            # AMCL 启动后也会发布 map->odom（可能产生 multiple authority 警告，但 map 能正常显示）
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir}.items()
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        condition=LaunchConfigurationEquals('localization', 'icp'),
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir}
                        ],
                        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
                    )
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_path,
                    'params_file': nav2_params_file_dir,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'spin_speed': 5.0 # rad/s
        }]
    )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_path,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz}.items()
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)

    ld.add_action(start_rm_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    ld.add_action(start_localization_group)
    ld.add_action(bringup_fake_vel_transform_node)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld
