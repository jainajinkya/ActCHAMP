# ActChamp
Action dependent CHAMP algorithm

## Data Collection Steps
1. Launch moveit ```roslaunch hlpr_jaco_moveit_config_two_arms_7dof hlpr_simple_moveit.launch```
2. Start force control model on leader arm ```rosservice call /right_arm_driver/in/start_force_control```
3. Launch Arm shadowing node ```rosrun arm_utils shadow_arm``` or ```rosrun arm_utils kinova_jogger``` [Preferred]
3. Launch SimTrack ```roslaunch simtrack_nodes <filename>.launch ```
4. Launch Data publisher nodes: ```roslaunch experiments data_topics.launch```
   - Check if the datatopics are getting published 
   ``` 
   rostopic echo /changepoint/pose_data
   rostopic echo /changepoint/action_data
   ```
5. Launch Rviz ```rviz```
6. Record data: ``` rosbag record -O ~/ajinkya_ws/src/active_champ/experiments/data/bagfiles/FILENAME.bag /changepoint/pose_data /changepoint/action_data /changepoint/reference_frame /changepoint/secondary_frame```


## Data Analysis Steps
1. Run data parser:  ``` roscd experiments/src/ &&
    python demo_data_parser.py FILENAME ```
   - `.bag` is added automatically in the script
   - Run rqt_bag to find threshold timestamps: ```rqt_bag```
 
 2. Run changepoint server ```roslaunch changepoint changepoint_server.launch ```
    - Adjust relevant parameters in the 'articulation.cpp' file to reflect errors in the data

 3. Make request
    - CHAMP request: 
    ``` roscd changepoint/test && python articTest_champ.py FILENAME ```

    - ActCHAMP request: ``` roscd changepoint/test && python articTest_champ.py FILENAME ```

    - NOTE: Adjust changepoint prior parameters in the files before making a request call.

4. Plotting comparison plots: 
    ```
    roscd changepoint/scripts
    python plot_loglikelihoods.py FILENAME
    ```


## Manipulation Experiments Steps
### Planner Setup
1. Set up POMDP Planner to use learned model parameters
  - Define problem defintion file in `pomdp_hd/src/pomdp_hd/src/problem_definitions/` folder
  - Update goal state model parameters in `pomdp_hd/src/pomdp_hd/src/belief_evolution.cpp` file
  - Compile belief_evolution cython file 
  ``` bash
  cd pomdp_hd/src/pomdp_hd/cython
  python setup.py build_ext --inplace
  ```
2. Create a script file to use the planner specific for the task 
3. Do a dry-run of planner to check if everything works

### Experimental Setup
1. Check planned target_frames published on the tf tree to ensure plan looks feasible and the estimated model parameters are reasonable.
2. Specific for the task, an additional task specific transform may need to be defined in order to manipulate objects. Define it in the `gen_coord_to_ee_target` function of `pomdp_hd/scripts/planner_interface_2.py` file.
3. Redo a dry-runto check if the plan looks reasonable.
4. Set the goal state and perform the experiments.

### Running Experiments
1. Open 5 Terminals.
2. Terminal# 1: Start the robot, if not running, `vstart`
3. Terminal# 2: Launch moveit for controlling arms `launchArms`
4. Terminal# 3: Source ajinkya_ws and launch feedback topics `saj && roslaunch experiments data_topics.launch`
5. Terminal# 4: Source ajinkya_ws and launch tf_frame broadcaster `saj && roslaunch experiments tf_broadcaster.launch`
6. Terminal# 5: Launch task script `cd pomdp_hd/scripts && python TASK_SCRIPT.py`
