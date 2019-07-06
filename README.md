# ActChamp
Package to use the ActCHAMP algorithm for data collection, changepoint analysis, and POMDP manipulation planning using learned models.

## Run Examples
### Changepoint Detection
- Run changepoint server ```roslaunch changepoint_detection changepoint_detection_server.launch ```
- Run detection examples for ActCHAMP ``` roscd changepoint/scripts && python actChamp_main.py ```
- To run a specific example:
  - Microwave with grasp: ``` roscd changepoint/scripts && python actChamp_main.py  dataset 1 ```
  - Microwave without grasp: ``` roscd changepoint/scripts && python actChamp_main.py dataset 2```
  - Drawer with grasp: ``` roscd changepoint/scripts && python actChamp_main.py dataset 3 ```
  - Drawer without grasp: ``` roscd changepoint/scripts && python actChamp_main.py dataset 4 ```
  - Stapler: ``` roscd changepoint/scripts && python actChamp_main.py dataset 5 ```

### Manipulation Planning
- Compile cython bindings
  - Update object specific parameter in  `pomdp_hd/src/pomdp_hd/src/belief_evolution.cpp` file
  - Compile belief_evolution cython file 
  ``` bash
  cd pomdp_hd/src/pomdp_hd/cython
  python setup.py build_ext --inplace
  ```
  - Compile it separately for each of the object. 

- Run planning examples
  - Drawer: ``` roscd pomdp_hd/scripts && python test_drawer.py ```
  - Microwave: ``` roscd  pomdp_hd/scripts && python test_microwave.py ```
  - Stapler: ``` roscd pomdp_hd/scripts && python test_stapler.py ```


## Repository Details
## Data Collection Steps: Experiments
1. Start impedance control mode for the leader arm.
2. Launch Arm shadowing node ```rosrun arm_utils shadow_arm``` [Moveit based shadowing] or ```rosrun arm_utils kinova_jogger``` [Direct joint control based shadowing, **Preferred**]
3. Launch SimTrack ```roslaunch simtrack_nodes <filename>.launch ``` or ar_track_alvar for tag based tracking ``` roslaunch experiments gemini_bundle.launch```
4. Launch Data publisher nodes: ```roslaunch experiments data_topics.launch```
   - Check if the datatopics are getting published 
   ``` 
   rostopic echo /changepoint/pose_data
   rostopic echo /changepoint/action_data
   ```
5. Launch Rviz ```rviz``` [optional]
6. Record data: ``` rosbag record -O ~/ajinkya_ws/src/active_champ/experiments/data/bagfiles/FILENAME.bag /changepoint/pose_data /changepoint/action_data /changepoint/reference_frame /changepoint/secondary_frame```


## Data Analysis Steps: Changepoint_detection
1. Run data parser:  ``` roscd experiments/src/ &&
    python demo_data_parser.py FILENAME ```
   - This generates a <FILENAME>.pkl file containing cleaned data to be used for changepoint analysis
   - Run rqt_bag to find threshold timestamps: ```rqt_bag``` and trim the data to generate noise-free data
   - Note: `.bag` is added automatically in the script 
 
 2. Run changepoint server ```roslaunch changepoint_detection changepoint_detection_server.launch ```
    - May need to update the observation data-likelihood parameters to reflect observational noise. Adjust relevant parameters in the *changepoint_detection/src/articulation.cpp* file and compile again.

 3. Make changepoint detection request
    - Generates a plot showing the detected changepoints and their governing models. Model parameters and detected changepoints are saved as "experiments/data/cp_data/<FILENAME>.txt"
    - CHAMP request: 
    ``` roscd changepoint/scripts && python champ_main.py <FILENAME> ```
    - ActCHAMP request: ``` roscd changepoint/scripts && python actChamp_main.py <FILENAME> ```
    - Note: May need to adjust changepoint prior parameters in the files to account for different experimental datasets.

4. Plotting comparison plots: 
    ```
    roscd changepoint/scripts
    python plot_loglikelihoods.py FILENAME
    ```

## Manipulation Planning
### Planner Setup
1. Set up POMDP Planner to use learned model parameters
  - Define problem defintion file in `pomdp_hd/src/pomdp_hd/src/problem_definitions/` folder
  - Update goal state model parameters in `pomdp_hd/src/pomdp_hd/src/belief_evolution.cpp` file
  - Compile belief_evolution cython file 
  ``` bash
  cd pomdp_hd/src/pomdp_hd/cython
  python setup.py build_ext --inplace
  ```
2. Create a script file to use the planner specific for the task. Examples can be found under pomdp_hd/scripts.
3. Feedback is available as rostopic at "/changepoint/pose_data/"
