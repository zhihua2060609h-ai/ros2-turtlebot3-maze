"""一键启动 Gazebo 迷宫、Burger 模型、RViz2 和控制节点。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    own=get_package_share_directory('maze_control')
    tb3=get_package_share_directory('turtlebot3_gazebo')
    gazebo=get_package_share_directory('gazebo_ros')
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in dict(
        world='maze',mode='wall',gui='true',rviz='true',adaptive='true',
        goal_x='1.0',goal_y='0.0',x='0.0',y='0.0',exit_x='4.4',
        exit_y_max='-0.65',max_duration='600.0').items()]
    args.append(DeclareLaunchArgument('yaw', default_value=PythonExpression([
        "'1.5708' if '", LaunchConfiguration('world'),
        "' == 'maze' else '0.0'"])))
    simulator=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo,'launch','gazebo.launch.py')),
        launch_arguments={'world':PathJoinSubstitution([own,'worlds',[LaunchConfiguration('world'),'.world']]),
                          'gui':LaunchConfiguration('gui'),'verbose':'false'}.items())
    # 直接读取 URDF，避免启动器给坐标系名称加斜杠，与 Gazebo 的 TF 保持一致。
    with open(os.path.join(tb3,'urdf','turtlebot3_burger.urdf'),encoding='utf-8') as f:
        robot_description=f.read()
    description=Node(package='robot_state_publisher',executable='robot_state_publisher',
        parameters=[{'use_sim_time':True,'robot_description':robot_description}])
    spawn=Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
        arguments=['-entity','burger','-file',os.path.join(tb3,'models','turtlebot3_burger','model.sdf'),
                   '-x',LaunchConfiguration('x'),'-y',LaunchConfiguration('y'),
                   '-Y',LaunchConfiguration('yaw'),'-z','0.01'])
    control=Node(package='maze_control',executable='controller',output='screen',
        parameters=[{'use_sim_time':True,'mode':LaunchConfiguration('mode'),
                     'adaptive':LaunchConfiguration('adaptive'),
                     'goal_x':LaunchConfiguration('goal_x'),'goal_y':LaunchConfiguration('goal_y'),
                     'exit_x':LaunchConfiguration('exit_x'),
                     'exit_y_max':LaunchConfiguration('exit_y_max'),
                     'max_duration':LaunchConfiguration('max_duration')}])
    rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(own,'rviz','maze.rviz')],
        parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription(args+[
        SetEnvironmentVariable('TURTLEBOT3_MODEL','burger'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),
        simulator,description,spawn,rviz,
        # 生成模型的进程退出后启动控制节点，由节点检查传感器数据是否就绪。
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=[control]))])
