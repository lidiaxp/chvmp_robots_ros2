<launch>
    <!-- Arguments -->
    <arg name="robot_name" default="/" description="Robot namespace."/>
    <arg name="rviz" default="false" description="Launch RViz if true."/>
    <arg name="frame_prefix" if="$(eval arg('robot_name') == '/')" value=""/>
    <arg name="frame_prefix" unless="$(eval arg('robot_name') == '/')" value="$(var robot_name)/"/>

    <group ns="$(var robot_name)">
        <!-- Gmapping for SLAM -->
        <include file="$(find-package spot_config)/launch/include/gmapping.launch.py">
            <arg name="frame_prefix" value="$(var frame_prefix)"/>
        </include>

        <!-- Calls navigation stack -->
        <include file="$(find-package spot_config)/launch/include/move_base.launch.py">
            <arg name="frame_prefix" value="$(var frame_prefix)"/>
            <arg name="robot_name" value="$(var robot_name)"/>
        </include>

        <!-- RViz Node -->
        <node if="$(var rviz)" pkg="rviz2" exec="rviz2" name="rviz" output="screen"
              args="-d package://champ_navigation/rviz/navigate.rviz -f $(var frame_prefix)map"/>
    </group>
</launch>