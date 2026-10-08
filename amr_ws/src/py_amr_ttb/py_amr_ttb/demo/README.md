# Lab 1 task demos

Standalone Lab 1 ROS nodes live here; shared helpers stay in the parent package.

| Module | Purpose |
| --- | --- |
| task1_teleop.py | Joystick demo |
| task2_wander.py | Wandering demo |
| task3_pid.py | PID demo |
| task4_state_machine.py | Integrated behavior demo |

Launch files remain in the package's `launch/` directory. Executable names and
launch commands are unchanged; setup.py resolves these modules through
`py_amr_ttb.demo`. Each standalone node publishes velocity, so run one at a time.

Lab 2 nodes, including Aiden's `lab2_go_to_goal.py`, live in the sibling `lab2/`
subpackage with their navigation logic. Shared-logic extraction is deferred.
