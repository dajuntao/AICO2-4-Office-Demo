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
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("- - - - - - START SYSTEM INITIALIZATION - - - - - -")
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("")

    logger.info("- - - - - - - ARMS INITIALIZATION - - - - - - - - -")
    arm_pair = connect_arm_pair(arm_L_sn, arm_R_sn, logger)
    logger.info("")

    logger.info("- - - - - - - - BASE INITIALIZATION - - - - - - - -")
    AMR_states, navigator = connect_AMR(AMR_ip, logger)
    logger.info("[Base] AMR is initialized.")
    logger.info("")

    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("- - - - - - - SYSTEM IS INITIALIZED - - - - - - - -")
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("")

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

    # check if the arms were operational already
    was_operational = arm_pair.operational()

    # Enable the pair of robots, make sure the E-stop is released before enabling
    logger.info("[Arm] Enabling arms ...")
    arm_pair.Enable()

    # Wait for the arm to become operational
    while not arm_pair.operational():
        time.sleep(1)
    logger.info("[Arm] Both arms are now operational\n")

    logger.info("- - - - - - - GRIPPERS INITIALIZATION - - - - - - -")
    # if the arms were not operational already, initialize grippers
    if not was_operational:
        logger.info("[Gripper] Initializing grippers ...")
        init_gripper(arm_pair, logger)
    logger.info("[Gripper] Both grippers are now initialized")

    return arm_pair


# initialize grippers
def init_gripper(arm_pair, logger): 
    gripper_pair = flexivdrdk.GripperPair(arm_pair)
    gripper_pair.Enable(["Flexiv-GN01", "Flexiv-GN01"])
    gripper_pair.Init()
    time.sleep(11)


# initialize Seer AMR API
def connect_AMR(AMR_ip, logger):
    AMR_states = flexivamr.StatesAPI()
    if (not AMR_states.connect(AMR_ip)):
        logger.warn("[Base] States failed to connect")
        return 1
    else:
        logger.info("[Base] States connected successfully")

    navigator = flexivamr.NavigatorAPI()
    if (not navigator.connect(AMR_ip)):
        logger.warn("[Base] Navigator failed to connect")
        return 1
    else:
        logger.info("[Base] Navigator connected successfully")

    control = flexivamr.ControlAPI()
    if (not control.connect(AMR_ip)):
        logger.warn("[Base] Control failed to connect")
        return 1
    else:
        logger.info("[Base] Control connected successfully")

    configure = flexivamr.ConfigureAPI()
    if (not configure.connect(AMR_ip)):
        logger.warn("[Base] Configure failed to connect")
        return 1
    else:
        logger.info("[Base] Configure connected successfully")

    # seize AMR control and check relocation
    init_AMR(AMR_states, control, configure, logger)
    
    return AMR_states, navigator


# seize AMR control and check relocation 
def init_AMR(states, control, configure, logger):
    logger.info("[Base] Initializing the mobile robot...")
    name = "MyAICO2-4"

    # gain control over amr
    bool_control = configure.gain_control(name)
    if bool_control:
        logger.info("[Base] Control seized successfully")
    else:
        logger.warn("[Base] Control failed to seize")
    
    # check if amr states::control is seized
    bool_control = states.is_control_seized()
    if bool_control:
        logger.info("[Base] Control state is seized")
    else:
        logger.warn("[Base] Control state is not seized")
    
    # clear faults [not implemented]
    #configure.clear_errors(name)

    # check relocation
    if states.check_relocation_status().reloc_status != 1:
        # execute relocation
        reloc = control.Relocation()

        # automatic relocation - AMR Version 3.4.6.18 and above
        reloc.is_auto = True

        control.perform_relocation(reloc)
        while states.check_relocation_status().reloc_status == 2:
            logger.info("[Base] relocalizing...")
            time.sleep(1)

        # AMR Version 3.4.6.18 and above do not require calling the confirmation positioning API again.
        # control.confirm_correct_location()  

    if states.check_relocation_status().reloc_status == 1:
        logger.info("[Base] AMR is relocalized")
    else:
        raise Exception("[Base] AMR cannot be relocalized")


# execute routines with arm plans and move Seer AMR
def execute_routines(arm_pair, AMR_states, navigator, arm_plans, logger):
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("- - - - - - - START RUNNING ROUTINES  - - - - - - -")
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("")
    
    try:
        # create two instances for both arms
        L_arm, R_arm = arm_pair.instances()

        # initialize gripper
        # execute_arm_plan([arm_plans['gripper_init'] + 'L', arm_plans['gripper_init'] + 'R'], arm_pair, logger)
        # logger.info("[Arm] Both Grippers are initialized.\n")

        # move arm to the home pose and reset gripper width
        execute_arm_plan([arm_plans['arm_homing'] + 'L', arm_plans['arm_homing'] + 'R'], arm_pair, logger)
        logger.info("[Arm] Reset Arms home pose and Grippers home width.")
        logger.info("")
        
        # move AMR to the universal plug-in station (LM3)
        move_AMR(AMR_states, navigator, arm_pair, logger, start="AP1", target="LM3")
        logger.info("[Base] Moved to plug-in station.")
        logger.info("")

        # calibrated work coordinate and synch the new work coordinate in both arms
        execute_arm_plan([arm_plans['work_coord_calib'] + 'L', arm_plans['wait'] + 'R'], arm_pair, logger)
        R_arm.SetGlobalVariables({'workCoord': arm_pair.global_variables()[0]['workCoord']}) # synch the left arm work coordinate with the right arm
        logger.info("[Arm] Work coordinate calibrated and synched.")
        logger.info("")

        ##################### [testing] arm pair and AMR fault stop behavior test ######################
        # execute_arm_plan([arm_plans['fault'] + 'L', arm_plans['pick-up'] + 'BNC_R'], arm_pair, logger)
        ################################################################################################

        # move arm to transition + pickup BNC
        execute_arm_plan([arm_plans['transition_coord'] + 'L', arm_plans['pick-up'] + 'BNC_R'], arm_pair, logger)
        logger.info("[Arm] Moved Left arm to transition pose and the right arm pick up BNC.")
        logger.info("")

        # move arm to pickup 500v + curl
        execute_arm_plan([arm_plans['pick-up'] + '500v_L', arm_plans['curl-up'] + 'R'], arm_pair, logger)
        logger.info("[Arm] Moved Left arm to pickup the 500v connector pose and the right arm curled-up to clear obstacles.")
        logger.info("")

        # move to plug-in + plug-in
        execute_arm_plan([arm_plans['plug-in'] + '500v_L', arm_plans['plug-in'] + 'BNC_R'], arm_pair, logger)
        logger.info("[Arm] Moved Left arm to plug in the 500v connector pose and the right arm pluged-in to BNC.")
        logger.info("")

        # move arm to the home pose and reset gripper width
        execute_arm_plan([arm_plans['arm_homing'] + 'L', arm_plans['arm_homing'] + 'R'], arm_pair, logger)
        logger.info("[Arm] Reset Arms home pose and Grippers home width.")
        logger.info("")

        # move AMR back to home (AP1)
        move_AMR(AMR_states, navigator, arm_pair, logger, start="LM3", target="AP1")
        logger.info("[Base] Moved back to home.")

    except Exception as e:
        if arm_pair.fault():
            raise Exception(f"At least one arm is faulted when running {e}")
        else:
            raise Exception(str(e))
    except KeyboardInterrupt:
        arm_pair.StopPlan()
        raise KeyboardInterrupt("The routine is canceled by the user.")
    
    print("", flush=True)
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - -")
    logger.info("- - - - - - - ROUTINES ARE FINISHED - - - - - - -")
    logger.info("- - - - - - - - - - - - - - - - - - - - - - - - -")


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
                # logger.info("[Arm] The arm is switched to the PLAN_EXECUTION mode.")
                break

        # execute plans by name
        logger.info(f"[Arm] Left Arm: execute {plan_names[0]}")
        logger.info(f"[Arm] Right Arm: execute {plan_names[1]}")
        arm_pair.ExecutePlan(plan_names, False)

        # print current plans output
        while arm_pair.busy() and not stop_event.is_set() and not arm_pair.fault():
            left_arm_plan_info, right_arm_plan_info = arm_pair.plan_info()
            logger.info(" ")
            # print(f"[Arm] event state: {str(stop_event.iss_set())}")
            # print(f"[Arm] fault state: {str(arm_pair.fault())}")
            # print(f"[Arm] busy state: {str(arm_pair.busy())}")
            print(f"[Arm] left_plan_name: {left_arm_plan_info.assigned_plan_name}")
            print(f"[Arm] left_pt_name: {left_arm_plan_info.pt_name}")
            print(f"[Arm] left_node_name: {left_arm_plan_info.node_name}")
            print(f"[Arm] right_plan_name: {right_arm_plan_info.assigned_plan_name}")
            print(f"[Arm] right_pt_name: {right_arm_plan_info.pt_name}")
            print(f"[Arm] right_node_name: {right_arm_plan_info.node_name}")
            print("", flush=True)
            time.sleep(1)
        
    # Thread for executing both arms plan
    plans_execute_thread = threading.Thread(target=plans_execute)
    plans_execute_thread.start()
    time.sleep(0.1)   # to wait till the arms are in busy state

    while arm_pair.busy() and not stop_event.is_set():
        # check if at least one of the arm is in fault. if so both arms are stopped and an Exception will be raised
        if arm_pair.fault():
            arm_pair.StopPlan()
            stop_event.set()
            raise Exception(f"\"{plan_names[0]}\" or \"{plan_names[1]}\"")
        time.sleep(0.1)

    # Wait for thread to exit
    plans_execute_thread.join()


# move Seer AMR
def move_AMR(states, navigator, arm_pair, logger, start, target):
    # set up a non-blocking input exit event for thread interruption
    exit_input = threading.Event()

    logger.info(f"[Base] Moving AMR from {str(start)} to {str(target)}")
    # multi-threading non-blocking user input function
    def non_blocking_input_handler():
        while (navi_states.task_status != 4 or navi_states.task_status != 6) and not exit_input.is_set(): # not finish and not canceled and event is not exited
            if select.select([sys.stdin], [], [], 1)[0]:    # check if sys.stdin is ready to be read with a timeout of 0 (non-blocking)
                interruption = sys.stdin.readline().strip()
                if interruption.lower() == 'p':
                    navigator.pause_current_navigation()
                    print("[Base] path navigation paused.")
                elif interruption.lower() == 'r':
                    navigator.continue_current_navigation()
                    print("[Base] path navigation resumed.s")
            else:    
                time.sleep(0.1)
    
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

    try:
        while ((navi_states.task_status == 2 or navi_states.task_status != 4) and navi_states.task_status != 6):
            if (arm_pair.fault() == True):
                navigator.cancel_current_navigation()
                raise Exception()
            
            navi_states = states.check_navigation_status()
            logger.info("")
            print(f"[Base] \'p + ENTER\' to pause")
            print(f"[Base] \'r + ENTER\' to resume")
            print(f"[Base] target wp: {str(navi_states.target_id)}")
            print(f"[Base] navi status: {str(navi_states.task_status)}")
            print("", flush=True)
            time.sleep(1)

    except KeyboardInterrupt: # use keyboardInterrupt to end the whole program
        navigator.cancel_current_navigation()
        exit_input.set()
        raise KeyboardInterrupt()

    # terminate thread and rejoin the thread
    exit_input.set()
    input_thread.join()  


# caller function
def main():
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
                    "wait": "AICO2_wait_",
                    "fault": "AICO2_fault_"
                }
    
    try:
        logger = spdlog.ConsoleLogger("AICO2 Universal Plug-in Demo")
        arm_pair, AMR_states, navigator = sys_init(arm_L_sn, arm_R_sn, AMR_ip, logger)

        # run AICO2 demo routines
        execute_routines(arm_pair, AMR_states, navigator, arm_plans, logger)

    except Exception as e:
        logger.error(str(e))
    except KeyboardInterrupt as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()