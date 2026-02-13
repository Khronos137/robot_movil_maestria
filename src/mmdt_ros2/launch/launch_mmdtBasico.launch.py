import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    # --- 1. ARGUMENTOS Y RUTAS ---

    # Declarar 'use_sim_time', lo forzamos a 'true' para esta simulación
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Seteamos 'use_ros2_control' a 'true' ya que lo estamos usando
    use_ros2_control = 'true' 

    # Rutas a los paquetes
    pkg_mmdt_ros2 = get_package_share_directory('mmdt_ros2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Ruta a tu XACRO principal (basado en tu rsp.launch.py)
    # ¡Asegúrate que tu robot.urdf.xacro esté en 'description/'!
    xacro_file = os.path.join(pkg_mmdt_ros2, 'description', 'robot.urdf.xacro')

    # --- 2. LÓGICA DE TU RSP.LAUNCH.PY (Integrada aquí) ---

    # Procesar el XACRO (esta es tu lógica de confianza)
    robot_description_config = Command(['xacro ', xacro_file, 
                                        ' use_ros2_control:=', use_ros2_control, 
                                        ' sim_mode:=', use_sim_time])
    
    # Parámetros para el Robot State Publisher
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    
    # Nodo Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # --- 3. LANZAR GAZEBO ---
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # --- 4. APARCAR (SPAWN) EL ROBOT EN GAZEBO ---
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', # Se suscribe al RSP
                   '-entity', 'mmdt_ros2',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'], # Posición inicial
        output='screen'
    )

    # --- 5. LANZAR LOS CONTROLADORES (SPAWNERS) ---
    # (Usando los nombres de tus archivos YAML)

    # A. Broadcaster (DEBE ser el primero en cargar)
    spawner_joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # B. Controlador de Ruedas
    spawner_ruedas_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['controlador_velocidad_ruedas', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # C. Controlador de Torso
    spawner_torso_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['torso_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # D. Controlador de Brazo Derecho
    spawner_manip_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manipulator_h_der_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # --- 6. ORQUESTACIÓN (El orden es vital) ---

    # Lanzar el Broadcaster DESPUÉS de que el robot aparezca en Gazebo
    handler_spawn_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=[spawner_joint_broad],
        )
    )

    # Lanzar los otros 3 controladores DESPUÉS de que el Broadcaster esté listo
    handler_spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_joint_broad,
            on_exit=[spawner_ruedas_controller, 
                     spawner_torso_controller, 
                     spawner_manip_controller],
        )
    )

    # --- 7. DEVOLVER LA DESCRIPCIÓN DEL LAUNCH ---
    return LaunchDescription([
        declare_sim_time_arg,
        launch_gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        handler_spawn_broadcaster,
        handler_spawn_controllers,
        # Como pediste, no se incluye RViz
    ])