#!/usr/bin/env python

"""AICO2-4_universal_plugin_demo.py
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import spdlog  # pip install spdlog
import select
import threading

# Import Flexiv DRDK and Seer AMR Python library
import sys
sys.path.insert(0, "./flexivamr_lib_py")
import flexivrdk
import flexivdrdk
import flexivamr


# connect and initialize arm and AMR
def sys_init(arm_L_sn, arm_R_sn, AMR_ip, logger):
    arm_pair = connect_arm_pair(arm_L_sn, arm_R_sn, logger)
    AMR_states, navigator = connect_AMR(AMR_ip, logger)

    return arm_pair, AMR_states, navigator


# initialize RDK and instantiate arm objects
def connect_arm_pair(arm_L_sn, arm_R_sn, logger):
    arm_pair = flexivdrdk.RobotPair([arm_L_sn, arm_R_sn])

    # Clear fault on the connected robot if any
    if arm_pair.fault():
        logger.warn("[Arm] Fault occurred on the connected arm, trying to clear ...")
        # Try to clear the fault on both robots
        result = arm_pair.ClearFault()
        # If fault is not cleared on both robots
        if not (result[0] and result[1]):
            logger.error("[Arm] Fault cannot be cleared, exiting ...")
            return 1
        logger.info("[Arm] Fault on the connected arm is cleared")

    # Enable the pair of robots, make sure the E-stop is released before enabling
    logger.info("Enabling arms ...")
    arm_pair.Enable()

    # Wait for the arm to become operational
    while not arm_pair.operational():
        time.sleep(1)

    logger.info("Both arms are now operational")

    return arm_pair


# initialize Seer AMR API
def connect_AMR(AMR_ip, logger):
    AMR_states = flexivamr.StatesAPI()
    if (not AMR_states.connect(AMR_ip)):
        logger.warn("states failed to connect")
        return 1
    else:
        logger.info("states connected successfully")

    navigator = flexivamr.NavigatorAPI()
    if (not navigator.connect(AMR_ip)):
        logger.warn("navigator failed to connect")
        return 1
    else:
        logger.info("navigator connected successfully")

    control = flexivamr.ControlAPI()
    if (not control.connect(AMR_ip)):
        logger.warn("control failed to connect")
        return 1
    else:
        logger.info("control connected successfully")

    configure = flexivamr.ConfigureAPI()
    if (not configure.connect(AMR_ip)):
        logger.warn("configure failed to connect")
        return 1
    else:
        logger.info("configure connected successfully")

    # seize AMR control and check relocation
    init_AMR(AMR_states, control, configure, logger)
    
    return AMR_states, navigator


# seize AMR control and check relocation 
def init_AMR(states, control, configure, logger):
    logger.info("Initializing the mobile robot...")
    name = "AMR1"

    # gain control over amr
    bool_control = configure.gain_control(name)
    if bool_control:
        logger.info("control seized successfully")
    else:
        logger.warn("control failed to seize")
    
    # check if amr states::control is seized
    bool_control = states.is_control_seized()
    if bool_control:
        logger.info("states::control is seized")
    else:
        logger.warn("states::control is not seized")
    
    # clear faults
    #configure.clear_errors(name)

    # check relocation
    if states.check_relocation_status().reloc_status != 1:
        # execute relocation
        reloc = control.Relocation()

        # automatic relocation - AMR Version 3.4.6.18 and above
        reloc.is_auto = True

        control.perform_relocation(reloc)
        while states.check_relocation_status().reloc_status == 2:
            logger.info("relocalizing...")
            time.sleep(1)

        # AMR Version 3.4.6.18 and above do not require calling the confirmation positioning API again.
        # control.confirm_correct_location()  

    if states.check_relocation_status().reloc_status == 1:
        logger.info("AMR is relocalized")
    else:
        raise Exception("AMR cannot be relocalized")


# execute routines with arm plans and move Seer AMR
def execute_routines(arm_pair, AMR_states, navigator, arm_plans, logger):
    try:
        # create two instances for both arms
        L_arm, R_arm = arm_pair.instances()

        # initialize gripper
        execute_arm_plan([arm_plans['gripper_init'] + 'L', arm_plans['gripper_init'] + 'R'], arm_pair, logger)
        logger.info("Both Grippers are initialized.")

        # move arm to the home pose and reset gripper width
        execute_arm_plan([arm_plans['arm_homing'] + 'L', arm_plans['arm_homing'] + 'R'], arm_pair, logger)
        logger.info("Reset Arms home pose and Grippers home width.")
        
        # move AMR to the universal plug-in station (LM3)
        move_AMR(AMR_states, navigator, logger, start="AP1", target="LM3")
        logger.info("Move to plug-in station.")

        # calibrated work coordinate and synch the new work coordinate in both arms
        execute_arm_plan([arm_plans['work_coord_calib'] + 'L', arm_plans['wait'] + 'R'], arm_pair, logger)
        R_arm.SetGlobalVariables({'workCoord': arm_pair.global_variables()[0]['workCoord']}) # synch the left arm work coordinate with the right arm
        logger.info("Work coordinate calibrated and synched.")

        # move arm to transition + pickup BNC
        execute_arm_plan([arm_plans['transition_coord'] + 'L', arm_plans['pick-up'] + 'BNC_R'], arm_pair, logger)
        logger.info("Moves Left arm to transition pose while the right arm picks up BNC.")

        # move arm to pickup 500v + curl
        execute_arm_plan([arm_plans['pick-up'] + '500v_L', arm_plans['curl-up'] + 'R'], arm_pair, logger)
        logger.info("Moves Left arm to pickup the 500v connector pose while the right arm curls up to clear obstacles.")

        # move to plug-in + plug-in
        execute_arm_plan([arm_plans['plug-in'] + '500v_L', arm_plans['plug-in'] + 'BNC_R'], arm_pair, logger)
        logger.info("Moves Left arm to plug in the 500v connector pose while the right arm plugs-in to BNC.")

        # move arm to the home pose and reset gripper width
        execute_arm_plan([arm_plans['arm_homing'] + 'L', arm_plans['arm_homing'] + 'R'], arm_pair, logger)
        logger.info("Reset Arms home pose and Grippers home width.")

        # move AMR back to home (AP1)
        move_AMR(AMR_states, navigator, logger, start="LM3", target="AP1")
        logger.info("Move back to home.")

    except KeyboardInterrupt:
        raise KeyboardInterrupt("Routine is canceled")


# execute arm plans
def execute_arm_plan(plan_names, arm_pair, logger):
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

    # Execute two plans from both arms
    def plans_execute():
        # switch to plan execution mode
        while True:
            try:
                arm_pair.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            except:
                arm_pair.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            else:
                logger.info("[Arm] The arm is switched to the PLAN_EXECUTION mode.")
                break

        # execute plans by name
        logger.info(f"Left Arm: execute {plan_names[0]}")
        logger.info(f"Right Arm: execute {plan_names[1]}")
        arm_pair.ExecutePlan(plan_names, False)

        # print current plans output
        while arm_pair.busy():
            left_arm_plan_info, right_arm_plan_info = arm_pair.plan_info()
            logger.info(" ")
            print(f"left_plan_name: {left_arm_plan_info.assigned_plan_name}")
            print(f"left_node_name: {left_arm_plan_info.node_name}")
            # print(f"left_pt_name: {left_arm_plan_info.pt_name}")
            print(f"right_plan_name: {right_arm_plan_info.assigned_plan_name}")
            print(f"right_node_name: {right_arm_plan_info.node_name}")
            # print(f"right_pt_name: {right_arm_plan_info.pt_name}")
            # print(f"node_path: {left_arm_plan_info.node_path}")
            # print(f"node_path_time_period: {left_arm_plan_info.node_path_time_period}")
            # print(f"node_path_number: {left_arm_plan_info.node_path_number}")
            # print(f"velocity_scale: {plan_left_arm_plan_infoinfo.velocity_scale}")
            # print(f"waiting_for_step: {left_arm_plan_info.waiting_for_step}")
            print("", flush=True)
            time.sleep(0.1)
    
    # Thread for executing both arms plan
    plans_execute_thread = threading.Thread(target=plans_execute)
    plans_execute_thread.start()

    # Use main thread to catch keyboard interrupt and exit thread
    try:
        while arm_pair.busy() and not stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        # Send signal to exit thread
        logger.info("Stopping plans execute thread")
        stop_event.set()

    # Wait for thread to exit
    plans_execute_thread.join()
    logger.info("Plans execute thread exited")


# move Seer AMR
def move_AMR(states, navigator, logger, start, target):
    # set up a non-blocking input exit event for thread interruption
    exit_input = threading.Event()

    # multi-threading non-blocking user input function
    def non_blocking_input_handler():
        while (navi_states.task_status != 4 or navi_states.task_status != 6) and not exit_input.is_set(): # not finish and not canceled and event is not exited
            if select.select([sys.stdin], [], [], 1)[0]:    # check if sys.stdin is ready to be read with a timeout of 0 (non-blocking)
                interruption = sys.stdin.readline().strip()
                if interruption.lower() == 'c':
                    navigator.cancel_current_navigation()
                    exit_input.set()    # exit the event for thread interruption
                elif interruption.lower() == 'p':
                    navigator.pause_current_navigation()
                    logger.info("path navigation paused")
                elif interruption.lower() == 'r':
                    navigator.continue_current_navigation()
                    logger.info("path navigation resumed")
            else:    
                time.sleep(0.1)

    logger.info("\'c + ENTER\' to cancel the current amr navigation")
    logger.info("\'p + ENTER\' to pause  ..........................")
    logger.info("\'r + ENTER\' to resume ..........................")
    
    # fixed path navigation
    path = navigator.PathNavigationCommand()
    path.source_id = start
    path.id = target

    # navigate from source to id
    navigator.fixed_path_navigation(path)
    navi_states = states.check_navigation_status()

    # start a thread for concurrent non-blocking user input function
    input_thread = threading.Thread(target=non_blocking_input_handler)
    input_thread.start()

    logger.info("navi start status: " + str(navi_states.task_status))
    try:
        while ((navi_states.task_status == 2 or navi_states.task_status != 4) and navi_states.task_status != 6):
            time.sleep(1)
            navi_states = states.check_navigation_status()
            logger.info("target wp: " + str(navi_states.target_id))
            logger.info("navi status: " + str(navi_states.task_status))
    except KeyboardInterrupt:
        navigator.cancel_current_navigation()
        exit_input.set()
        raise KeyboardInterrupt()

    # terminate thread and rejoin the thread
    exit_input.set()
    input_thread.join()

    # check navigation cancel status to terminate the execution
    if navi_states.task_status == 6:
        raise KeyboardInterrupt("path navigation canceled")    


# caller function
def main():
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

    arm_L_sn = "Rizon4-063048"
    arm_R_sn = "Rizon4R-062059"
    AMR_ip = "192.168.1.110"

    arm_plans = {
                    "gripper_init": "AICO2_gripper_init_", 
                    "arm_homing": "AICO2_arm_init_",
                    "work_coord_calib": "AICO2_workCoord_calib_",
                    "transition_coord": "AICO2_transition_",
                    "pick-up": "AICO2_pick-up_",
                    "plug-in": "AICO2_plug-in_",
                    "curl-up": "AICO2_curl-up_",
                    "wait": "AICO2_wait_"
                }
    
    try:
        logger = spdlog.ConsoleLogger("AICO2 Universal Plug-in Demo")
        robot_pair, AMR_states, navigator = sys_init(arm_L_sn, arm_R_sn, AMR_ip, logger)

        # run AICO2 demo routines
        execute_routines(robot_pair, AMR_states, navigator, arm_plans, logger)

    except Exception as e:
        logger.error(str(e))
    except KeyboardInterrupt as e:
        logger.error(str(e) + " by the user.")


if __name__ == "__main__":
    main()