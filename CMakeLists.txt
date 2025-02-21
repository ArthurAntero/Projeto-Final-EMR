cmake_minimum_required(VERSION 3.5)

project(projeto-final-pkg)

# Definir o padrão C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Encontrar pacotes ROS2 necessários
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # Adicionando std_msgs
find_package(behaviortree_cpp_v3 REQUIRED)

# Declarar o executável do projeto com todos os arquivos .cpp do diretório src/
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/deliver-dish-to-kitchen.cpp
    src/deliver-to-table.cpp
    src/fetch-room.cpp
    src/fetch-room-for-dishes.cpp
    src/go-to-kitchen.cpp
    src/go-to-room.cpp
    src/go-to-waiting-room.cpp
    src/got-right-meal.cpp
    src/is-battery-above-15.cpp
    src/pickup-dish.cpp
    src/pickup-meal.cpp
    src/wait-for-call.cpp
    src/wait-for-pickup.cpp
)

# Linkar as bibliotecas do ROS2 e do BehaviorTree
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  geometry_msgs
  std_msgs
  BT::behaviortree_cpp_v3
)

# Definir a instalação do executável
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
