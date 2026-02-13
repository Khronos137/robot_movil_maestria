cmake_minimum_required(VERSION 3.8)
project(mmdt_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev REQUIRED) # Para los headers de Gazebo
find_package(gazebo_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # ¡AGREGA ESTA LÍNEA!

# Incluir carpeta include/
include_directories(include)

# ---------------------------------------------------------
# 2. Compilar el Nodo de Localización y Control (SetVels)
# ---------------------------------------------------------
add_executable(localization_setvels_node
  src/localization_setvels.cpp
  src/localization_utils.cpp
)
ament_target_dependencies(localization_setvels_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------
# 3. Compilar Plugins de Gazebo (Ghost Teleport)
# ---------------------------------------------------------
# Nota: Asumo que ghost_teleport es un plugin de Gazebo. 
# Si da error, comenta esta sección hasta que revisemos ese código.
# add_library(ghost_teleport SHARED
#   src/plugins/ghost_teleport.cpp
# )
# ament_target_dependencies(ghost_teleport
#   rclcpp
#   gazebo_dev
#   gazebo_ros
# )

# ---------------------------------------------------------
# 4. Instalación (Mover archivos al directorio de install)
# ---------------------------------------------------------
# Instalar ejecutables y librerias
install(TARGETS
  localization_setvels_node
#   ghost_teleport
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar carpetas de recursos (CRUCIAL para .launch.py, .yaml, .xacro, .stl)
install(DIRECTORY
  config
  description
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Instalar headers (si otros paquetes los necesitan)
install(DIRECTORY include/
  DESTINATION include
)




ament_package()