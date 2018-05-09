# -*- coding:utf-8 -*-
'''
功能说明: 目的是生成launch文件,用字符串来表示各个组件,然后
来实现拓扑的变换吧.
'''
# -*- coding:utf8 -*-
import string
import math
import copy

OUTPUT_PATH = '../launch/'
OUTPUT_NAME = 'many'
OUTPUR_SUFFIX = '_.launch'

class MyTemplate(string.Template):
    '''
    修改原来的Template
    '''
    delimiter = '%'

class Topology(object):
    '''
        Topology parse
    '''
    def __init__(self, topology_str, pose=[]):
        self.topology_str_ = topology_str
        self.topology_arr_ = []
        self.topology_ = []
        self.topology_size_ = 0
        self.topology_set_ = []
        self.single_str_ = ''
        self.pose=pose
        self.main_dict={'single_templates':'', 'mav_name':'hummingbird'}
        #     def generate_single(self, relative_templates, base_x=0.0, base_y=0.0, base_z=0.0,
        #  index=0, other_index=1,
        #  is_leader=True, mav_name='hummingbird', model_name='hummingbird', take_off_height=3,
        #  control_use_true_value=True, feature_type='orb'):
        self.single_dict={'relative_templates' : '', 'base_x' : 0, 'base_y' : 0, 'base_z' : 0, 
        'index' : 0, 'leader_index' : 0, 'is_leader' : True, 'mav_name' : 'hummingbird', 
        'model_name' : 'hummingbird', 'take_off_height' : 3.0, 'control_use_true_value' : False,
        'feature_type' : 'orb' , 'real_joy' : False
        }        

        # 参数： feature_type, index, other_index, is_leader, mav_name
        self.relative_dict={ 'index' : 0, 'other_index' : 0, 
        'mav_name' : 'hummingbird', 'feature_type' : 'orb' , 'is_self_control' : False
        }
        self.main_template_ = '''
<launch>
  <arg name="world_name" default="map3_world"/>
  <!-- <arg name="log_file" default="%{mav_name}" />-->
  <arg name="feature_type" default="orb" />
  <arg name="control_use_true_value" default="true"/>
  <!-- The following line causes gzmsg and gzerr messages to be printed to the console
      (even when Gazebo is started through roslaunch) -->

    %{single_templates}
</launch>
'''
        # 需要的参数：relative_templates, base_x, base_y, base_z, index, leader_index, mav_name, model_name, take_off_height, control_use_true_value, is_leader
        self.single_template_ = '''
  <group ns="%{mav_name}%{index}">    
    <!--dji sdk 节点-->
    <node pkg="dji_sdk" type="dji_sdk_node" name="dji_sdk" output="screen">
      <!-- node parameters -->
      <param name="serial_name" type="string" value="/dev/ttyUSB0"/>
      <param name="baud_rate" type="int" value="230400"/>
      <param name="app_id" type="int" value="1049073" />
      <param name="app_version" type="int" value="1"/>
      <param name="align_time" type="bool" value="false"/>
      <param name="enc_key" type="string" value="a11aebe98da42b84156aa99969c329ccee0b349e4be8995cdd3c3cb1ffef0b8a"/>
      <param name="use_broadcast" type="bool" value="false"/>
    </node>

    <!--dji adapter 节点-->
    <node pkg="rotors_utils" type="dji_adapter" name="dji_adapter" output="screen" />

    <!--相机节点, 本来是飞机的一部分来着-->
    <node pkg="uvc_camera" type="uvc_camera_node" name="camera">
    </node>

    <!--svo 节点-->
    <include file="$(find svo_ros)/launch/test_uav.launch">
      <arg name="cam_topic" value="image_raw" />
    </include>

    %{relative_templates}

    <!--控制器 -->
    <node name="lee_position_controller_node" pkg="rotors_control" type="lee_position_controller_node" output="screen" >
      <rosparam command="load" file="$(find rotors_utils)/params/lee_controller_%{model_name}.yaml" />
      <rosparam command="load" file="$(find rotors_utils)/params/%{model_name}.yaml" />
      <remap from="odometry" to="dji_odometry"/>
      <param name="take_off_height" value="%{take_off_height}-2" />
    </node>

    <!--和手柄通信的节点-->
    <node name="joy_node" pkg="joy" type="joy_node" output="screen" if="%{real_joy}">
      <param name="dev" value="/dev/input/js0" />
    </node>    
    
    <!--用键盘模拟出的手柄节点-->
    <node name="joy_sim" pkg="rotors_utils" type="joy_sim" output="screen" unless="%{real_joy}"> 
    </node>

    <!--从hover_control修改过来,变得能够使用手柄控制-->
    <!--或许这边应该换个控制器?-->
    <!--算了不用了,改手柄的command_control吧-->
    <node name="joy_pose" pkg="rotors_utils" type="joy_pose"  output="screen" >
      <param name="my_id" value="%{index}"/>
      <param name="leader_id" value="%{leader_index}"/>
      <param name="is_follower" value="%{is_follower}"/>
      <param name="take_off_height" value="%{take_off_height}" />
      <param name="follower_pose" value="/%{mav_name}%{leader_index}/follower_pose%{leader_index}%{index}" />
      <param name="target_pose" value="/%{mav_name}%{index}/target_pose"/>
      <param name="receive_image" value="/%{mav_name}%{leader_index}/receive_image%{leader_index}%{index}"  />
    </node>
    
    <!--手柄的具体动作-->
    <node name="joy_control" pkg="rotors_gazebo" type="joy_control" output="screen" >
    </node>

    <!--轨迹平滑器-->
    <node name="buffer" pkg="rotors_utils" type="buffer" output="screen">
    </node>

    <!--功能是各种信息数据类型的转化-->
    <node name="transformation" pkg="rotors_utils" type="transformation" output="screen" >
      <param name="mag_sub_frame" value="magnetometer" />
      <param name="mag_pub_frame" value="mag" />

      <param name="filtered_imu_sub_frame" value="filtered_imu" />
      
      <param name="pressure_sub_frame" value="air_pressure" />
      <param name="baro_pub_frame" value="altimeter" />

      <param name="odometry_pub_frame" value="odometry_pub_frame" />
      <param name="odometry_sub_frame" value="odometry_sensor1/odometry" />

      <param name="estimate_pose_frame" value="estimate_pose" />
      <param name="estimate_velocity" value="estimate_velocity" />
      <param name="estimate_rate" value="estimate_rate" />
      
      <param name="svo_sub_frame" value="svo/pose" />
      <param name="svo_pub_frame" value="svo/fusion_pose" />
    </node>

    <!--估计飞机的各种参数-->
     <node pkg="pose_estimator" type="pose_estimation" name="position_estimation" output="screen" >
      <rosparam file="$(find pose_estimator)/params/simulation.yaml" />
      <param name="nav_frame" value="/nav" />
      <param name="publish_world_nav_transform" value="true" />
      <remap from="raw_imu" to="imu" />
      <remap from="svo_image" to="svo/fusion_pose" />
      <param name="tf_prefix" value=""/>
      <remap from="image_pos_vel" to="image_pub_frame" />
      <remap from="magnetic" to="mag" />
    </node>

  </group>
'''
        # 参数： feature_type, index, other_index, is_leader, mav_name, 
        self.relative_template_ = '''
  <!--relative start for %{index} %{other_index}!-->
  <!--通过接收两副图像，来确定相对位置，接受的参数是：拓扑结构，自身节点的消息-->
  <node name="%{feature_type}_main%{index}%{other_index}" pkg="sift"
   type="%{feature_type}" output="screen">
    <!--载入拓扑结构数据-->
    <param name="uav_index" value="%{index}"/>
    <param name="other_index" value="%{other_index}"/>
    <param name="receive_image" value="receive_image%{index}%{other_index}" />
  </node>

  <!--产生在L2中看F2的目标轨迹, 但是这个是F2产生的，问问老板改不改吧-->
  <node name="follower_pose%{index}%{other_index}" pkg="rotors_utils" 
    type="follower_pose" output="screen" >
    <param name="my_id" value="%{index}"/>
    <param name="other_id" value="%{other_index}"/>
    <param name="relative_pose" value="relative_pose%{index}%{other_index}" />
    <param name="follower_pose" value="follower_pose%{index}%{other_index}" />
  </node>

  <!--follower节点的期望值计算节点-->
  <node name="trajectory_generation%{index}%{other_index}" 
    pkg="rotors_utils" type="trajectory_generation" output="screen" >
    <param name="my_id" value="%{index}"/>
    <param name="follower_id" value="%{other_index}"/>
    <param name="is_self_control" value="%{is_self_control}"/>
    <param name="T_L2_F2" value="/%{mav_name}%{index}/relative_pose%{index}%{other_index}"/>
    <param name="T_LS_L2" value="/%{mav_name}%{index}/svo/fusion_pose"/>
    <param name="T_FS_F2" value="/%{mav_name}%{other_index}/svo/fusion_pose"/>
    <param name="T_L2_dF2" value="/%{mav_name}%{index}/leader_desired_pose%{index}%{other_index}"/>
    <param name="T_FW_dF2" value="/%{mav_name}%{other_index}/target_pose"/>
  </node>
  <!--relative end for %{index} %{other_index}!-->
'''
        
    # relative_templates, base_x, base_y, base_z, index, other_index,
    #  mav_name, model_name, take_off_height, control_use_true_value, is_leader
    def generate_single(self, relative_templates, base_x=0.0, base_y=0.0, base_z=0.0, index=0, leader_index=1,
         is_follower=True, mav_name='hummingbird', model_name='hummingbird', take_off_height=3,
         control_use_true_value=False, feature_type='orb', target_pose_index=0, real_joy=True):
        '''
        生成单个飞机的launch段,
        完成这个之前需要先完成relative
        '''
        single_dict = copy.deepcopy(self.single_dict)
        single_dict['relative_templates']=relative_templates
        single_dict['base_x']=base_x
        single_dict['base_y']=base_y
        single_dict['base_z']=base_z
        single_dict['index']=index
        single_dict['leader_index']=leader_index
        single_dict['is_follower']=is_follower
        single_dict['mav_name']=mav_name
        single_dict['model_name']=model_name
        single_dict['take_off_height']=take_off_height
        single_dict['control_use_true_value']=control_use_true_value
        single_dict['feature_type']=feature_type
        single_dict['target_pose_index']=target_pose_index
        single_dict['real_joy']=real_joy
        return MyTemplate(self.single_template_).substitute(single_dict)    

    def generate_relative(self, index, other_index,is_self_control=False, mav_name='hummingbird', feature_type='orb'):
        '''
        生成获取相对位置的launch段
        '''
        rel_dict=copy.deepcopy(self.relative_dict)
        rel_dict['index']=index
        rel_dict['other_index']=other_index
        rel_dict['is_self_control']=is_self_control
        rel_dict['mav_name']=mav_name
        rel_dict['feature_type']=feature_type
        return MyTemplate(self.relative_template_).substitute(rel_dict)    

    def generate_main(self):
        main_dict = copy.deepcopy(self.main_dict) 
        main_dict['single_templates'] = self.single_str_
        return MyTemplate(self.main_template_).substitute(main_dict)

    def write_file(self):
        '''
        要写三个文件
        一个是many_svo
        一个是single
        一个是relative
        '''
        launch = open(OUTPUT_PATH + 'launch' + OUTPUR_SUFFIX,'w')
        launch.write(self.str_)
        launch.close()
        pass
    
    def parse(self):
        '''
        parse
        首先分隔字符串，得到数组大小
        然后解析出拓扑大小，也就是飞机个数
        再分析出是leader还是follower，以及隶属关系是怎样的
        '''
        self.topology_arr_ = self.topology_str_.split()
        self.topology_size_ = int(math.sqrt(len(self.topology_arr_)))
        assert self.topology_size_ ** 2 == len(self.topology_arr_)
        self.topology_.clear()
        for i in range(0, len(self.topology_arr_)):
            if self.topology_arr_[i] == '1':
                self.topology_set_.append((i%self.topology_size_,i//self.topology_size_))
        
        self.relative_strs_ = [''] * self.topology_size_
        self.single_strs_ = [''] * self.topology_size_

    def test_leader(self, index):
        '''
        检验是不是leader,如果是的话,返回follower的序号
        如果不是,返回0
        '''
        for i in self.topology_set_:
            if i[0] == index:
                return i[1]
        return 0
    
    def test_follower(self, index):
        '''
        检验是不是follower,如果是的话,返回leader的序号
        如果不是,返回-1
        '''
        for i in self.topology_set_:
            if i[1] == index:
                return i[0]
        return 0
    

    def config(self):
        for i in range(0,len(self.topology_set_)):
            # 先识别relatives
            leader_index, follower_index = self.topology_set_[i]
            is_self_control = False
            if(leader_index == follower_index):
                is_self_control = True
            self.relative_strs_[leader_index] += self.generate_relative(leader_index,
                follower_index, is_self_control=is_self_control)

        for i in range(0,len(self.relative_strs_)):
            self.relative_strs_[i] = self.relative_strs_[i].replace('True','true').replace('False','false')

        self.single_str_ = ''
        # 把relatives放进single中
        for i in range(0,len(self.single_strs_)):
            # relative_templates, base_x, base_y, base_z, index, leader_index, is_follower, 
            #  mav_name, model_name, take_off_height, control_use_true_value, is_leader
            index = i
            leader_index = self.test_follower(index)
            is_follower = (leader_index != index)
            self.single_strs_[i] = self.generate_single(relative_templates=self.relative_strs_[i],
                        base_x=self.pose[i][0], 
                        # base_y=(i-1)*1.2, 
                        base_y=self.pose[i][1], 
                        is_follower=is_follower, index=index,
                        leader_index=leader_index, real_joy=False
                        ).replace('True','true').replace('False','false')
            self.single_str_ += self.single_strs_[i]

        # 最后替换掉main launch文件里的singles_templates
        self.str_ = self.generate_main()
        
    def run(self):
        '''
        主要流程
        先解析
        然后设置configure
        '''
        self.parse()
        self.config()
        self.write_file()
    
    def run_without_write(self):
        self.parse()
        self.config()


def main():
    '''
    main
    想把仿真和实际都做到一个文件里去
    实际只能用一个飞机,所以还是分开吧
    '''
    # top_ = Topology('1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0'
    #     ,[[0,0],[1.2,1.2],[-1.2,-1.2],[1.2,-1.2],[-1.2,1.2]])
    # top_ = Topology('0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0',[[0,0],[1.2,1.2],[-1.2,-1.2],[1.2,-1.2]])
    # top_ = Topology('1 0 0 1 0 0 1 0 0',[[0,0],[1.2,1.2],[-1.2,1.2]])
    # top_ = Topology('0 0 0 1 0 0 0 1 0',[[-1.2,-1.2],[0,0],[1.2,1.2]])
    # top_ = Topology('1 0 1 0',[[0,0],[1.2,1.2]])
    top_ = Topology('0',[[0,0]])
    top_.run()
    # print(top_.relative_strs_)
    # print(top_.generate_single(2, 0, False))

if __name__ == '__main__':
    main()
