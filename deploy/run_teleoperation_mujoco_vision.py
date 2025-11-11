from deploy.config import Config
from deploy.controllers.controller import Runner_online_real_dexhand, Runner_handle_mujoco_vision #, Runner_offline_mujoco

import time
import cv2
from multiprocessing import shared_memory
import threading
from deploy.teleop.open_television.tv_wrapper import TeleVisionWrapper
from deploy.teleop.robot_control.robot_arm_ik import G1_29_ArmIK
from deploy.teleop.image_server.image_client import ImageClient

import numpy as np
import torch
torch.set_printoptions(precision=3)
np.set_printoptions(precision=3)

tv_img_shape = (480, 1280, 3)
tv_img_dtype = np.uint8
tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
tv_img_array = np.ndarray(tv_img_shape, dtype = tv_img_dtype, buffer = tv_img_shm.buf)

# client = ImageClient(image_show=True, server_address='192.168.123.164', Unit_Test=False)  # deployment test
# client.receive_process()

# img_client = ImageClient(tv_img_shape=tv_img_shape,
#                          tv_img_shm_name=tv_img_shm.name)
#                         #  server_address='10.7.127.146', 
#                         #  port=8012)  # , server_address='10.100.6.192', port=8012
# # img_client = ImageClient(image_show=True,tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name)
# image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
# image_receive_thread.start()

# television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
tv_wrapper = TeleVisionWrapper(True, tv_img_shape, tv_img_shm.name)

# arm_ctrl = G1_29_ArmController()
arm_ik = G1_29_ArmIK()

def deploy_real(args):
    # Load config
    config_path = f"deploy/configs/{args.config}"
    print(config_path)
    config = Config(config_path)

    runner = Runner_online_real_dexhand(config, args=args)
    # Enter the zero torque state, press the start key to continue executing
    runner.damping_state()
    # Enter the default position state, press the A key to continue executing
    runner.move_to_default_pos()
    runner.last_control_timestamp = time.time()
    current_mode = "SQUAT"
    print('Squat mode!')
    print('Press Left_Joystick to start the locomotion mode!')
    def tv_arms():
        head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.get_data()
        current_lr_arm_q = runner.qj.copy()[15:29]
        current_lr_arm_dq = runner.dqj.copy()[15:29]
        # # solve ik using motor data and wrist pose, then use ik results to control arms.
        sol_q, sol_tauff = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
        # print(f"sol_q: {sol_q}")
        print(left_wrist, right_wrist)
        sol_q = np.clip(sol_q, runner.target_dof_pos[runner.config.action_hl_idx] - 0.01,
                        runner.target_dof_pos[runner.config.action_hl_idx] + 0.01)
        runner.target_dof_pos[runner.config.action_hl_idx] = sol_q

    while True:
        # tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
        # cv2.imshow("record image", tv_resized_image)
        # key = cv2.waitKey(1) & 0xFF
        if current_mode == "LOCOMOTION":
            # print('Locomotion mode!')
            # runner.run_hand(debug=args.debug, manual=False)
            runner.run_loco(debug=args.debug, manual=True)
            if runner.transfer_to_squat:
                current_mode = "SQUAT"
                print('Squat mode!')
                print('Press Left_Joystick to start the locomotion mode!')

        elif current_mode == "SQUAT":
            # print('Squat mode!')
            # runner.run_hand(debug=args.debug, manual=False)
            # tv_arms()
            runner.run_squat_hand(debug=args.debug, manual=True)
            if runner.transfer_to_loco:
                current_mode = "LOCOMOTION"
                print('Locomotion mode!')
                print('Press Right_Joystick to start the squat mode!')
        # runner.run_homie(debug=args.debug, manual=True)

        
    tv_img_shm.unlink()
    tv_img_shm.close()
    print("Finally, exiting program...")
    exit(0)

def deploy_handle_mujoco(args):
    import mujoco.viewer
    import mujoco
    config_path = f"deploy/configs/{args.config}"
    print(config_path)
    config = Config(config_path)


    # Initialize DDS communication
    runner = Runner_handle_mujoco_vision(config, args=args)
    current_mode = "SQUAT"
    print('Squat mode!')
    print('Press Left_Joystick to start the locomotion mode!')
    def tv_arms():
        head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.get_data()
        current_lr_arm_q = runner.qj.copy()[15:29]
        current_lr_arm_dq = runner.dqj.copy()[15:29]
        # # solve ik using motor data and wrist pose, then use ik results to control arms.
        sol_q, sol_tauff = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
        # print(f"sol_q: {sol_q}")
        sol_q = np.clip(sol_q, runner.target_dof_pos[runner.config.action_hl_idx] - 0.01,
                        runner.target_dof_pos[runner.config.action_hl_idx] + 0.01)
        runner.target_dof_pos[runner.config.action_hl_idx] = sol_q

    with mujoco.viewer.launch_passive(runner.m, runner.d) as viewer:
        runner.last_control_timestamp = time.time()
        while True:
            # tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
            # cv2.imshow("record image", tv_resized_image)
            # key = cv2.waitKey(1) & 0xFF
            if current_mode == "LOCOMOTION":
                runner.run_loco(manual=True)
                viewer.sync()
                if runner.transfer_to_squat:
                    current_mode = "SQUAT"
                    print('Squat mode!')
                    print('Press Left_Joystick to start the locomotion mode!')

            elif current_mode == "SQUAT":
                tv_arms()
                # runner.target_dof_pos[runner.config.action_hl_idx] = np.array(runner.config.manual_upper_pos0, dtype=np.float32)[runner.config.action_hl_idx]
                runner.run_squat(manual=True)
                viewer.sync()
                if runner.transfer_to_loco:
                    current_mode = "LOCOMOTION"
                    print('Locomotion mode!')
                    print('Press Right_Joystick to start the squat mode!')
            # runner.run_homie(manual=True)
            # viewer.sync()

            # 传入图片属于是仿真/本地渲染生成的，可以传入读取到的实时画面吗？ todo注释 使用sharememory
            np.copyto(tv_img_array, np.array(runner.render_image))
            cv2.imshow("camera_view", cv2.cvtColor(runner.render_image, cv2.COLOR_RGB2BGR))
            # cv2.waitKey(1)

            #想要展现实时画面的话：下面的会是一片漆黑，其实是无法进行实时的显示的 自己修改的部分
            # cv2.imshow("camera_view", cv2.cvtColor(tv_img_array, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

    tv_img_shm.unlink()
    tv_img_shm.close()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--net", type=str, help="network interface")
    parser.add_argument("--config", type=str, help="config file name in the configs folder", default="run_loco_squat_grasp.yaml")
    parser.add_argument("--save_data", action="store_true", help="whether saving the real data")
    parser.add_argument("--save_data_dir", type=str, help="where to save the data", default="./save_real_data")
    parser.add_argument("--record_odometry", action="store_true", help="whether recording odometry")
    parser.add_argument("--odometry_topic_name", type=str, help="odometry topic name", default="/Odometry")
    parser.add_argument("--debug", action="store_true", help="")
    args = parser.parse_args()

    # deploy_real(args)
    deploy_handle_mujoco(args)
