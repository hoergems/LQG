gcc 4.8.3 required

This package implements LQG-MP (Van der Berg et. al.) for a simple 3-link manipulator toy problem.

Use the config file to configure the parameters. Running "python lqg.py" will create a set of RRT paths in joint space from 
the start to the goal state (this can be a simple linear path when the parameter 'use_linear_paths' is set to True).
For each covariance value (determined by the lower covariance bound 'min_covariance', the upper covariance bound 'max_covariance', 
and the number of covariance values 'covariance_steps'), 'number_of_generated_paths'-paths are created and saved in a 'paths.yaml' file.
If the parameter 'overwrite_paths_file' is set to true, a new paths.yaml file will be created and the old one is overwritten.

If a 'paths.yaml' file already exists and the parameter 'use_paths_from_file' is set to True, no new paths will be created. Instead
the already existing paths froms the paths.yaml file are used.

These paths are then evaluated for each process covariance value and the best path for each covariance value will be saved in a 'best_paths.yaml' file. After the paths have been evaluated for each covariance value, 'num_simulation_runs'-simulation runs are performed.
For each simulation run, the resulting end effector will be collected. After the simulations have finished for each covariance value, the
resulting end effector positions are saved inside a 'cartesian_coordinates.yaml' file.

In order to plot several statstics such as the average distance of the end effector positions to the goal position for each covariance value,
the histograms and EMD (Earth Mover's distance) plot, run 'python plot_stats.py save'. This will generate the plots inside the 'histograms' folder.
