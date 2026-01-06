#!/usr/bin/env python

"""AMR-arm_demo_demoRoom.py
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import spdlog  # pip install spdlog
import select
import threading

# Import Flexiv RDK and Seer AMR Python library
import sys
sys.path.insert(0, "../lib_py")
import flexivdrdk
import flexivamr


# connect and initialize arm and AMR
def sys_init(arm_sn, AMR_ip, logger):
    arm = connect_arm(arm_sn, logger)
    AMR_states, navigator = connect_AMR(AMR_ip, logger)

    return arm, AMR_states, navigator


# initialize RDK and instantiate arm objects
def connect_arm(arm_sn, logger):
    arm = flexivrdk.Robot(arm_sn)

    # Clear fault on arm server if any
    if arm.fault():
        logger.warn("Fault occurred on arm server, trying to clear ...")
        # Try to clear the fault
        arm.clearFault()
        time.sleep(2)
        # Check again
        if arm.fault():
            logger.error("Fault cannot be cleared, exiting ...")
            return
        logger.info("Fault on arm server is cleared.")

    # Enable the arm, make sure the E-stop is released before enabling
    logger.info("Enabling arm ...")
    arm.Enable()

    # Wait for the arm to become operational
    while not arm.operational():
        time.sleep(1)

    logger.info("Arm is now operational.")

    return arm


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

        # manual recolation with AMR location
        reloc.x = 2.0
        reloc.y = -3.7
        reloc.angle = 3.14159

        # automatic relocation - AMR Version 3.4.6.18 and above
        # reloc.is_auto = True

        control.perform_relocation(reloc)
        while states.check_relocation_status().reloc_status == 2:
            logger.info("relocalizing...")
            time.sleep(1)

        # AMR Version 3.4.6.18 and above do not require calling the confirmation positioning API again.
        control.confirm_correct_location()  

        if states.check_relocation_status().reloc_status == 1:
            logger.info("AMR is relocalized")
        else:
            raise Exception("AMR cannot be relocalized")


# execute routines with arm plans and move Seer AMR
def execute_routines(arm, AMR_states, navigator, logger):
    # move arm to the home pose and reset gripper
    execute_arm_plan('AMR_demo_gripper_init', arm, logger)
    logger.info("Gripper initialized.")

    # move arm to the home pose and reset gripper
    execute_arm_plan('AMR_demo_arm_init', arm, logger)
    logger.info("Arm reseted.")
    
    # move AMR to the insertion location (LM4)
    move_AMR(AMR_states, navigator, logger, start="LM1", target="LM3")
    logger.info("Move to plug-in station.")

    # calibrated work coordinate
    execute_arm_plan('AMR_demo_workCoord_cali', arm, logger)
    logger.info("Work coordinate calibrated.")

    # perform insertion
    execute_arm_plan('AMR_demo_insert_USB', arm, logger)
    logger.info("Finish USB insertion.")

    # reset arm
    execute_arm_plan('AMR_demo_arm_init', arm, logger)
    logger.info("Reset arm.")

    # move AMR back to home (LM1)
    move_AMR(AMR_states, navigator, logger, start="LM3", target="LM1")
    logger.info("Move back to home.")


# execute arm plans
def execute_arm_plan(name, arm, logger):
    # switch to plan execution mode
    arm.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)

    # execute plan by name
    logger.info(f"Arm: execute {name}")
    arm.ExecutePlan(name, False)

    # print current plan
    while arm.busy():
        plan_info = arm.plan_info()
        logger.info(" ")
        print(f"assigned_plan_name: {plan_info.assigned_plan_name}")
        print(f"pt_name: {plan_info.pt_name}")
        print(f"node_name: {plan_info.node_name}")
        print(f"node_path: {plan_info.node_path}")
        print(f"node_path_time_period: {plan_info.node_path_time_period}")
        print(f"node_path_number: {plan_info.node_path_number}")
        print(f"velocity_scale: {plan_info.velocity_scale}")
        print(f"waiting_for_step: {plan_info.waiting_for_step}")
        print("", flush=True)
        time.sleep(1)


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
    while ((navi_states.task_status == 2 or navi_states.task_status != 4) and navi_states.task_status != 6):
        time.sleep(1)
        navi_states = states.check_navigation_status()
        logger.info("target wp: " + str(navi_states.target_id))
        logger.info("navi status: " + str(navi_states.task_status))

    # terminate thread and rejoin the thread
    exit_input.set()
    input_thread.join()

    # check navigation cancel status to terminate the execution
    if navi_states.task_status == 6:
        raise KeyboardInterrupt("path navigation canceled")    


# caller function
def main():
    arm_sn = "Rizon4-062143"
    AMR_ip = "192.168.1.2"
    
    try:
        logger = spdlog.ConsoleLogger("AMR Demo")
        arm, AMR_states, navigator = sys_init(arm_sn, AMR_ip, logger)
        execute_routines(arm, AMR_states, navigator, logger)

    except Exception as e:
        logger.error(str(e))
    except KeyboardInterrupt as e:
        logger.error(str(e) + " by user")


if __name__ == "__main__":
    main()