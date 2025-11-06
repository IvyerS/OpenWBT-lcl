import time
import sys
import struct
import threading
from typing import Dict, Callable, Optional

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class ButtonStateMachine:
    """
    按钮状态机 - 每次上升沿切换状态
    """
    
    def __init__(self, button_names: list):
        self.button_names = button_names
        # 当前状态 (0或1)
        self.current_states = {name: 0 for name in button_names}
        # 前一个状态，用于边沿检测
        self.previous_states = {name: 0 for name in button_names}
        # 状态变化回调函数
        self.callbacks: Dict[str, Optional[Callable[[str, int], None]]] = {}
        print(f"初始化按钮状态机，支持 {len(button_names)} 个按钮")
        
    def update_button(self, button_name: str, current_value: int):
        """
        更新按钮状态
        button_name: 按钮名称
        current_value: 当前读取值 (0或1)
        """
        if button_name not in self.button_names:
            return
            
        prev_value = self.previous_states[button_name]
        
        # 检测上升沿 (0->1)
        if current_value == 1 and prev_value == 0:
            # 切换状态: 0->1 或 1->0
            old_state = self.current_states[button_name]
            self.current_states[button_name] = 1 - self.current_states[button_name]
            
            print(f"🔘 {button_name}: 状态 {old_state}→{self.current_states[button_name]}")
            
            # 触发回调
            if button_name in self.callbacks and self.callbacks[button_name]:
                self.callbacks[button_name](button_name, self.current_states[button_name])
        
        # 更新前一个状态
        self.previous_states[button_name] = current_value
    
    def get_state(self, button_name: str) -> int:
        """获取指定按钮的当前状态"""
        return self.current_states.get(button_name, 0)
    
    def set_callback(self, button_name: str, callback: Callable[[str, int], None]):
        """设置状态变化回调函数"""
        
        if button_name in self.button_names:
            self.callbacks[button_name] = callback
    
    def get_all_states(self) -> Dict[str, int]:
        """获取所有按钮状态"""
        return self.current_states.copy()
    
    def reset_all(self):
        """重置所有按钮状态"""
        self.current_states = {name: 0 for name in self.button_names}
        self.previous_states = {name: 0 for name in self.button_names}
        print("所有按钮状态已重置")

class EnhancedUnitreeService:
    def __init__(self):
        # 摇杆数据 (实时值)
        self.Lx = 0.0           
        self.Rx = 0.0            
        self.Ry = 0.0            
        self.Ly = 0.0

        # 按钮名称列表 (Unitree遥控器的所有按钮)
        button_names = [
            'L1', 'L2', 'R1', 'R2', 
            'A', 'B', 'X', 'Y', 
            'Up', 'Down', 'Left', 'Right', 
            'Select', 'F1', 'F3', 'Start'
        ]
        
        #控制信号值
        self.start_signal = False
        self.run_signal = False
        self.run_loco_signal = False
        self.run_squat_signal = False
        self.damping_signal = False
        self.stopgait_signal = False
        self.left_hand_grasp_state = True
        self.right_hand_grasp_state = True

        # 初始化按钮状态机
        self.button_state_machine = ButtonStateMachine(button_names)
        
        # 设置按钮回调
        # self._setup_button_callbacks()
        print("控制器已初始化")

    def parse_buttons(self, data1: int, data2: int):
        """
        解析按钮数据并更新状态机
        使用状态机逻辑：只在按下时切换状态
        """
        # 解析data1的各个按钮
        buttons_data1 = {
            'R1': (data1 >> 0) & 1,
            'L1': (data1 >> 1) & 1,
            'Start': (data1 >> 2) & 1,
            'Select': (data1 >> 3) & 1,
            'R2': (data1 >> 4) & 1,
            'L2': (data1 >> 5) & 1,
            'F1': (data1 >> 6) & 1,
            'F3': (data1 >> 7) & 1
        }
        
        # 解析data2的各个按钮
        buttons_data2 = {
            'A': (data2 >> 0) & 1,
            'B': (data2 >> 1) & 1,
            'X': (data2 >> 2) & 1,
            'Y': (data2 >> 3) & 1,
            'Up': (data2 >> 4) & 1,
            'Right': (data2 >> 5) & 1,
            'Down': (data2 >> 6) & 1,
            'Left': (data2 >> 7) & 1
        }
        
        # 更新所有按钮的状态机
        for button_name, value in buttons_data1.items():
            self.button_state_machine.update_button(button_name, value)
        
        for button_name, value in buttons_data2.items():
            self.button_state_machine.update_button(button_name, value)

    def parse_joysticks(self, data: bytes):
        """
        解析摇杆数据 (保持实时值)
        """
        lx_offset = 4
        self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

    def parse(self, remoteData: bytes):
        """
        解析完整的远程控制器数据
        """
        # 解析摇杆数据 (实时)
        self.parse_joysticks(remoteData)
        
        # 解析按钮数据 (使用状态机)
        self.parse_buttons(remoteData[2], remoteData[3])
        
        # 打印调试信息
        self._print_debug_info()

    def _print_debug_info(self):
    #     """打印调试信息"""
    #     # 摇杆状态
    #     print("摇杆状态 (实时值):")
    #     print(f"  Lx: {self.Lx:.3f}, Ly: {self.Ly:.3f}")
    #     print(f"  Rx: {self.Rx:.3f}, Ry: {self.Ry:.3f}")
        # 按钮状态
        print("按钮状态 (切换状态):")
        button_states = self.button_state_machine.get_all_states()
        for i, (button, state) in enumerate(button_states.items()):
            print(f"  {button}: {state}", end="")
            if (i + 1) % 4 == 0:  # 每4个按钮换行
                print()
        print()

    def get_button_state(self, button_name: str) -> int:
        """获取指定按钮的当前状态"""
        return self.button_state_machine.get_state(button_name)
    
    def get_joystick_values(self) -> Dict[str, float]:
        """获取所有摇杆值"""
        return {'Lx': self.Lx, 'Ly': self.Ly, 'Rx': self.Rx, 'Ry': self.Ry }

class EnhancedCustom:
    def __init__(self):
        self.low_state = None 
        self.remoteController = EnhancedUnitreeService()

    def Init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        wireless_remote_data = self.low_state.wireless_remote
        # 使用增强版的解析方法
        self.remoteController.parse(wireless_remote_data)
        
        # 这里可以添加基于按钮状态的逻辑
        self._handle_button_joystick_actions()

    def _handle_button_joystick_actions(self):

        joystick_values = self.remoteController.get_joystick_values()

        """基于按钮状态执行相应动作"""
        # 示例：如果A按钮状态为1，执行某个功能
        if self.remoteController.get_button_state('A') == 1:
            self.run_loco_signal = True
            self.stopgait_signal = True
            print('Run loco signal: ', self.run_loco_signal)
        elif self.remoteController.get_button_state('A') == 0:
            self.run_loco_signal = False
            print('Run loco signal: ', self.run_loco_signal)

        if self.remoteController.get_button_state('B') == 1:
            self.stopgait_signal = not self.stopgait_signal

        # 按下X按钮启动，转移到初始化模式；按下Start按钮进入policy运行模式
        if self.remoteController.get_button_state('X') == 1 \
            and self.remoteController.get_button_state('Start')==0:
            self.start_signal = True
            print('Start signal: ', self.start_signal)

        elif self.remoteController.get_button_state('X') == 1 \
            and self.remoteController.get_button_state('Start')==1:
            self.run_signal = True
            print('Running signal: ', self.run_signal)

        elif self.remoteController.get_button_state('X') == 0:
            self.start_signal = False
            self.run_signal = False
            print('Start signal: ', self.start_signal)
            print('Running signal: ', self.run_signal)

        if self.remoteController.get_button_state('Y') == 1:
            self.stopgait_signal = not self.stopgait_signal
        
        #按下Down按钮，转为damping mode
        #再次按下Down按钮，退出damping mode

        if self.remoteController.get_button_state('Down') == 1:
            self.damping_signal = True
            print('Zero torque signal: ', self.damping_signal)
        elif self.remoteController.get_button_state('Down') == 0:
            self.damping_signal = False
            print('Zero torque signal: ', self.damping_signal)

        #按下Up按钮，转为squat mode
        if self.remoteController.get_button_state('Up') == 1:
            self.run_squat_signal = True
            print('Run squat signal: ', self.run_squat_signal)
        elif self.remoteController.get_button_state('Up') == 0:
            self.run_squat_signal = False
            print('Run squat signal: ', self.run_squat_signal)
        
        #使用了摇杆去控制手抓状态 替换原来的扳机按钮 这里默认手抓会进行闭合
        if joystick_values['Rx'] > 0.9 \
            and joystick_values['Ry'] < 0.5 \
            and joystick_values['Ry'] > -0.5:
            self.right_hand_grasp_state = True

        elif joystick_values['Rx'] < 0.2 \
            and joystick_values['Ry'] < 0.5 \
            and joystick_values['Ry'] > -0.5:
            self.right_hand_grasp_state = False

        if joystick_values['Lx'] > 0.9 \
            and joystick_values['Ly'] < 0.5 \
            and joystick_values['Ly'] > -0.5:
            self.left_hand_grasp_state = True

        elif joystick_values['Lx'] < 0.2 \
            and joystick_values['Ly'] < 0.5 \
            and joystick_values['Ly'] > -0.5:
            self.left_hand_grasp_state = False

# 测试函数
def test_enhanced_controller():
    """测试增强版控制器"""
    print("测试增强版Unitree遥控器控制器")
    
    # 创建控制器实例
    controller = EnhancedUnitreeRemoteController()
    
    # 模拟遥控器数据
    print("\n1. 模拟A按钮按下:")
    # 模拟A按钮按下 (data2的第0位为1)
    controller.parse_buttons(0, 1)  # data1=0, data2=1 (A按钮按下)
    
    print("\n2. 模拟A按钮释放:")
    # 模拟A按钮释放 (data2的第0位为0)
    controller.parse_buttons(0, 0)  # data1=0, data2=0 (A按钮释放)
    
    print("\n3. 模拟A按钮再次按下:")
    # 模拟A按钮再次按下
    controller.parse_buttons(0, 1)
    
    print("\n4. 模拟多个按钮同时操作:")
    # 模拟A和B按钮同时按下 (data2 = 00000011 = 3)
    controller.parse_buttons(0, 3)
    
    # 查看所有按钮状态
    print("\n最终按钮状态:")
    states = controller.button_state_machine.get_all_states()
    for button, state in states.items():
        if state == 1:  # 只显示状态为1的按钮
            print(f"  {button}: {state}")

if __name__ == '__main__':
    # # 运行测试
    # test_enhanced_controller()
    
    # 主程序
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = EnhancedCustom()
    custom.Init()

    try:
        while True:   
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序已停止")     
            
