# Lab 2

Lab 2 nodes and their navigation logic live together here for now.
Aiden owns [lab2_go_to_goal.py](lab2_go_to_goal.py); its odometry and control callbacks still need
implementation. DJ will extract shared logic later.

Shared configuration remains in `py_amr_ttb/lab_config.py`. Launch files remain
in the package's `launch/` directory. The executable mapping is:

```text
lab2_go_to_goal = py_amr_ttb.lab2.lab2_go_to_goal:main
```

After rebuilding and sourcing the Ubuntu workspace, the launch command is unchanged:

```bash
ros2 launch py_amr_ttb lab2_go_to_goal.launch.py
```

This starts the current skeleton, not a completed navigation demonstration.

See the [team work plan](../../../../../README.md#lab-2-work-plan), [Word plan](../../../../../docs/Lab_2_Team_Work_Plan.docx), and [task 1 launch file](../../launch/lab2_go_to_goal.launch.py).
