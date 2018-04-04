# -*- coding:utf8 -*-
'''
功能说明: 目的是生成launch文件,用字符串来表示各个组件,然后
来实现拓扑的变换吧.
'''
# -*- coding:utf8 -*-
import string

OUTPUT_PATH = '../launch/'
OUTPUT_NAME = 'three'
OUTPUR_SUFFIX = '_auto_gen.launch'

class MyTemplate(string.Template):
    '''
    修改原来的Template
    '''
    delimiter = '%'

class Topology(object):
    '''
        Topology parse
    '''
    def __init__(self):
        self.main_template_ = '''
<!-- this is for svo -->
<launch>
  <arg name="mav_index1" default="0"/>
  <arg name="mav_index2" default="-1"/>
  <arg name="mav_index3" default="2"/>
  <arg name="world_name" default="map3_world"/>
  <!-- <arg name="log_file" default="$(arg mav_name)" />-->
  <arg name="debug" default="false"/>
  <arg name="gui" default="true"/>
  <arg name="paused" default="false"/>
  <arg name="feature_type" default="orb" />
  <arg name="control_use_true_value" default="true"/>
  <!-- The following line causes gzmsg and gzerr messages to be printed to the console
      (even when Gazebo is started through roslaunch) -->
  <arg name="verbose" default="false"/>

  <env name="GAZEBO_MODEL_PATH" value="${GAZEBO_MODEL_PATH}:$(find rotors_gazebo)/models"/>
  <env name="GAZEBO_RESOURCE_PATH" value="${GAZEBO_RESOURCE_PATH}:$(find rotors_gazebo)/models"/>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find rotors_gazebo)/worlds/$(arg world_name).world" />
    <arg name="debug" value="$(arg debug)" />
    <arg name="paused" value="$(arg paused)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="verbose" value="$(arg verbose)"/>
  </include>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rotors_gazebo)/rviz/sim_two_svo.rviz"/>

  <include file="$(find rotors_utils)/launch/single_svo.launch">
    <arg name="index" value="$(arg mav_index1)" />
    <arg name="follower_index" value="$(arg mav_index2)" />
    <arg name="follower_index_2" value="$(arg mav_index3)" />
    <arg name="x" value="0"/>
    <arg name="y" value="0"/>
    <arg name="z" value="0.1"/>
    <arg name="feature_type" value="$(arg feature_type)"/>
    <arg name="is_leader" value="true" />
  </include>

</launch>

'''
        self.single_template_ = '''
 <include file="$(find rotors_utils)/launch/single_svo.launch">
    <arg name="index" value="$(arg mav_index2)" />
    <arg name="follower_index" value="$(arg mav_index1)" />
    <arg name="x" value="%{base_x}"/>
    <arg name="y" value="%{base_y}"/>
    <arg name="z" value="%{base_z}"/>
    <arg name="feature_type" value="%{feature_type}"/>
    <arg name="is_leader" value="%{is_leader}" />
  </include>
        '''
        self.relative_template_ = '''
<!--相对位置相关的节点在此-->
<launch>
  <arg name="mav_name" default="hummingbird"/>
  <arg name="feature_type" default="orb"/>
  <arg name="is_leader" default="false" />
  <arg name="index" default="0"/>
  <arg name="other_index" default="1"/>

  <!--通过接收两副图像，来确定相对位置，接受的参数是：拓扑结构，自身节点的消息-->
  <node name="$(arg feature_type)_main$(arg index)$(arg other_index)" pkg="sift"
   type="$(arg feature_type)" output="screen" if="$(arg is_leader)">
    <!--载入拓扑结构数据-->
    <param name="uav_index" value="$(arg index)"/>
    <param name="other_index" value="$(arg other_index)"/>
    <rosparam command="load" file="$(find rotors_utils)/params/topology.yaml" />
  </node>

  <!--产生在L2中看F2的目标轨迹, 但是这个是F2产生的，问问老板改不改吧-->
  <node name="follower_pose$(arg index)$(arg other_index)" pkg="rotors_utils" 
    type="follower_pose" output="screen" if="$(arg is_leader)" >
    <param name="my_id" value="$(arg index)"/>
    <param name="other_id" value="$(arg other_index)"/>
    <param name="relative_pose" value="relative_pose$(arg index)$(arg other_index)" />
    <param name="is_leader" value="$(arg is_leader)" />
  </node>

  <!--follower节点的期望值计算节点-->
  <node name="trajectory_generation$(arg index)$(arg other_index)" 
    pkg="rotors_utils" type="trajectory_generation" output="screen" if="$(arg is_leader)">
    <param name="my_id" value="$(arg index)"/>
    <param name="follower_id" value="$(arg other_index)"/>
    <param name="T_L2_F2" value="/$(arg mav_name)$(arg index)/relative_pose$(arg index)$(arg other_index)"/>
    <param name="T_LS_L2" value="/$(arg mav_name)$(arg index)/svo/fusion_pose"/>
    <param name="T_FS_F2" value="/$(arg mav_name)$(arg other_index)/svo/fusion_pose"/>
    <param name="T_L2_dF2" value="/$(arg mav_name)$(arg index)/leader_desired_pose$(arg index)$(arg other_index)"/>
    <param name="T_FW_dF2" value="/$(arg mav_name)$(arg other_index)/target_pose"/>
  </node>

</launch>
        '''

    def get_topology(self):
        '''
        hello
        '''
        pass

    def generate_single(self, base_x, base_y, is_leader, base_z='0', feature_type='orb'):
        '''
        生成单个飞机的launch段
        '''
        sub = {'base_x':str(base_x), 'base_y':str(base_y), 'base_z':str(base_z),
               'feature_type':str(feature_type), 'is_leader':str(is_leader)}
        return MyTemplate(self.single_template_).substitute(sub)

    def generate_relative(self, base_x, base_y, is_leader, base_z='0', feature_type='orb'):
        '''
        生成单个飞机的launch段
        '''
        sub = {'base_x':str(base_x), 'base_y':str(base_y), 'base_z':str(base_z),
               'feature_type':str(feature_type), 'is_leader':str(is_leader)}
        return MyTemplate(self.single_template_).substitute(sub)

    def parse(self):
        '''
        parse
        '''
        pass

def main():
    '''
    main
    '''
    top_ = Topology()
    print(top_.generate_single(2, 0, False))



if __name__ == '__main__':
    main()
