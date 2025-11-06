import time
import sys
import struct
import threading
from typing import Dict, Callable, Optional, Any

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class ButtonStateMachine:
    """
    按钮状态机 - 每次上升沿切换状态
    """
    
    def __init__(self, button_names: list):
        self.button_names = button_names
        self.current_states = {name: 0 for name in button_names}
        self.previous_states = {name: 0 for name in button_names}
        self.callbacks: Dict[str, Optional[Callable[[str, int], None]]] = {}
        print(f"初始化按钮状态机，支持 {len(button_names)} 个按钮")
        
    def update_button(self, button_name: str, current_value: int):
        if button_name not in self.button_names:
            return
            
        prev_value = self.previous_states[button_name]
        
        # 检测上升沿 (0->1)
        if current_value == 1 and prev_value == 0:
            old_state = self.current_states[button_name]
            self.current_states[button_name] = 1 - self.current_states[button_name]
            
            print(f"🔘 {button_name}: 状态 {old_state}→{self.current_states[button_name]}")
            
            # 触发回调
            if button_name in self.callbacks and self.callbacks[button_name]:
                self.callbacks[button_name](button_name, self.current_states[button_name])
        
        self.previous_states[button_name] = current_value
    
    def get_state(self, button_name: str) -> int:
        return self.current_states.get(button_name, 0)
    
    def set_callback(self, button_name: str, callback: Callable[[str, int], None]):
        if button_name in self.button_names:
            self.callbacks[button_name] = callback
    
    def get_all_states(self) -> Dict[str, int]:
        return self.current_states.copy()
    
    def reset_all(self):
        self.current_states = {name: 0 for name in self.button_names}
        self.previous_states = {name: 0 for name in self.button_names}
        print("所有按钮状态已重置")

class EnhancedUnitreeController:
    def __init__(self):
        # 摇杆数据
        self.Lx = 0.0           
        self.Rx = 0.0            
        self.Ry = 0.0            
        self.Ly = 0.0

        # 按钮名称列表
        button_names = [
            'L1', 'L2', 'R1', 'R2', 
            'A', 'B', 'X', 'Y', 
            'Up', 'Down', 'Left', 'Right', 
            'Select', 'F1', 'F3', 'Start'
        ]
        
        # 控制信号值
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
        
        # 全局回调列表
        self.global_callbacks = []
        
        print("Unitree控制器已初始化")

    def parse_buttons(self, data1: int, data2: int):
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
        
        for button_name, value in buttons_data1.items():
            self.button_state_machine.update_button(button_name, value)
        
        for button_name, value in buttons_data2.items():
            self.button_state_machine.update_button(button_name, value)

    def parse_joysticks(self, data: bytes):
        lx_offset = 4
        self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

    def parse(self, remoteData: bytes):
        self.parse_joysticks(remoteData)
        self.parse_buttons(remoteData[2], remoteData[3])
        
        # 触发全局回调
        for callback in self.global_callbacks:
            try:
                callback(self)
            except Exception as e:
                print(f"全局回调执行错误: {e}")

    def get_button_state(self, button_name: str) -> int:
        return self.button_state_machine.get_state(button_name)
    
    def get_joystick_values(self) -> Dict[str, float]:
        return {'Lx': self.Lx, 'Ly': self.Ly, 'Rx': self.Rx, 'Ry': self.Ry }
    
    def register_button_callback(self, button_name: str, callback: Callable[[str, int], None]):
        self.button_state_machine.set_callback(button_name, callback)
    
    def register_global_callback(self, callback: Callable[['EnhancedUnitreeController'], None]):
        """注册全局回调，每次解析数据时都会调用"""
        self.global_callbacks.append(callback)
    
    def get_all_signals(self) -> Dict[str, Any]:
        """获取所有信号状态"""
        return {
            'start_signal': self.start_signal,
            'run_signal': self.run_signal,
            'run_loco_signal': self.run_loco_signal,
            'run_squat_signal': self.run_squat_signal,
            'damping_signal': self.damping_signal,
            'stopgait_signal': self.stopgait_signal,
            'left_hand_grasp_state': self.left_hand_grasp_state,
            'right_hand_grasp_state': self.right_hand_grasp_state,
            'joystick': self.get_joystick_values(),
            'buttons': self.button_state_machine.get_all_states()
        }

class EnhancedCustom:
    def __init__(self, controller: EnhancedUnitreeController):
        self.low_state = None 
        self.controller = controller
        self.lowstate_subscriber = None
        self.is_running = False

    def Init(self):
        """初始化订阅器"""
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        self.is_running = True

    def LowStateMessageHandler(self, msg: LowState_):
        if not self.is_running:
            return
            
        self.low_state = msg
        wireless_remote_data = self.low_state.wireless_remote
        self.controller.parse(wireless_remote_data)
        self._handle_button_joystick_actions()

    def _handle_button_joystick_actions(self):
        if not self.is_running:
            return
        joystick_values = self.controller.get_joystick_values()

        # 按钮逻辑处理
        if self.controller.get_button_state('A') == 1:
            self.controller.run_loco_signal = True
            self.controller.stopgait_signal = True
        elif self.controller.get_button_state('A') == 0:
            self.controller.run_loco_signal = False
        if self.controller.get_button_state('B') == 1:
            self.controller.stopgait_signal = not self.controller.stopgait_signal

        # X + Start 组合逻辑
        if self.controller.get_button_state('X') == 1:
            if self.controller.get_button_state('Start') == 0:
                self.controller.start_signal = True
                self.controller.run_signal = False
            else:
                self.controller.start_signal = True
                self.controller.run_signal = True
        else:
            self.controller.start_signal = False
            self.controller.run_signal = False

        if self.controller.get_button_state('Y') == 1:
            self.controller.stopgait_signal = not self.controller.stopgait_signal
        
        #  UP - damping mode
        if self.controller.get_button_state('Up') == 1:
            self.controller.damping_signal = True
        else:
            self.controller.damping_signal = False

        # Down - squat mode
        if self.controller.get_button_state('Down') == 1:
            self.controller.run_squat_signal = True
        else:
            self.controller.run_squat_signal = False
        
        # 摇杆控制手抓状态
        if joystick_values['Rx'] > 0.9:
            self.controller.right_hand_grasp_state = True
        elif joystick_values['Rx'] < 0.2:
            self.controller.right_hand_grasp_state = False

        if joystick_values['Lx'] > 0.9:
            self.controller.left_hand_grasp_state = True
        elif joystick_values['Lx'] < 0.2:
            self.controller.left_hand_grasp_state = False

    def stop(self):
        """停止订阅器"""
        self.is_running = False
        # 注意：Unitree SDK 目前没有提供取消订阅的方法
        # 我们只能通过设置标志位来阻止进一步处理消息
        print("EnhancedCustom 已停止")

class UnitreeControllerService:
    """
    Unitree控制器服务 - 后台运行模式
    支持真正的启动和停止功能
    """
    def __init__(self, config_path: str = None):
        self.config_path = config_path
        self.controller = EnhancedUnitreeController()
        self.custom = None
        self.is_running = False
        self.control_thread = None
        self._stop_event = threading.Event()
        
    def start(self):
        """启动服务"""
        if self.is_running:
            print("服务已经在运行")
            return False
            
        print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
        # 初始化ChannelFactory
        try:
            if self.config_path:
                ChannelFactoryInitialize(0, self.config_path)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            print(f"ChannelFactory初始化失败: {e}")
            return False
            
        # 创建EnhancedCustom实例
        self.custom = EnhancedCustom(self.controller)
        self.custom.Init()
        
        self.is_running = True
        self._stop_event.clear()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("Unitree控制器服务已启动")
        return True
        
    def _control_loop(self):
        """控制循环 - 在后台线程中运行"""
        print("控制线程开始运行")
        while self.is_running and not self._stop_event.is_set():
            try:
                # 这里可以添加周期性的控制逻辑
                # 目前主要是保持线程运行并定期检查停止信号
                time.sleep(0.1)
            except Exception as e:
                print(f"控制循环错误: {e}")
                break
        print("控制线程结束")
            
    def stop(self):
        """停止服务"""
        if not self.is_running:
            return
        print("正在停止Unitree控制器服务...")
        
        self.is_running = False
        self._stop_event.set()
        
        # 停止EnhancedCustom
        if self.custom:
            self.custom.stop()
        
        # 等待控制线程结束
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=5.0)
            if self.control_thread.is_alive():
                print("警告: 控制线程未能及时停止")
        
    def register_button_callback(self, button_name: str, callback: Callable[[str, int], None]):
        """注册按钮回调"""
        self.controller.register_button_callback(button_name, callback)
        
    def register_global_callback(self, callback: Callable[[EnhancedUnitreeController], None]):
        """注册全局回调"""
        self.controller.register_global_callback(callback)
        
    def get_controller(self) -> EnhancedUnitreeController:
        """获取控制器实例"""
        return self.controller
        
    def get_status(self) -> Dict[str, Any]:
        """获取服务状态"""
        return {
            'is_running': self.is_running,
            'signals': self.controller.get_all_signals() if self.controller else {}
        }

# 使用示例和测试
def main():
    """主程序 - 服务模式运行"""
    # 创建服务实例
    service = UnitreeControllerService()
    
    # 定义回调函数
    def on_button_press(button_name: str, state: int):
        print(f"🎮 按钮 {button_name} 状态: {state}")
        if button_name == 'Start' and state == 1:
            print("🚀 开始执行任务!")
        elif button_name == 'A' and state == 1:
            print("🏃 进入运动模式!")
    
    def global_update_callback(controller: EnhancedUnitreeController):
        # 每次数据更新时都会调用
        signals = controller.get_all_signals()
        if signals['run_signal']:
            print("📡 检测到运行信号...")
        else:
            print("📡 未检测到运行信号...")
        if signals['start_signal']:
            print("📡 检测到启动信号...")
        else:
            print("📡 未检测到启动信号...")
        if signals['damping_signal']:
            print("📡 检测到阻尼模式信号...")
        else:
            print("📡 未检测到阻尼模式信号...")
        if signals['run_squat_signal']:
            print("📡 检测到下蹲模式信号...")
        else:
            print("📡 未检测到下蹲模式信号...")
        if signals['left_hand_grasp_state']:
            print("🤚 左手抓取状态: 抓取中...")
        if signals['right_hand_grasp_state']:
            print("🤚 右手抓取状态: 抓取中...")

    # 注册回调-查看年signals变化
    # service.register_global_callback(global_update_callback)
    
    # 启动服务
    if service.start():
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
        finally:
            # 确保服务被停止
            service.stop()
            
        print("程序结束")
    else:
        print("服务启动失败")

if __name__ == '__main__':
    main()