^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multidim_rrt_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2024-02-05)
------------------
* Bugfixes after renaming. Builds and launches fine.
* Rename to package name change
* Contributors: daviddorf2023

0.0.5 (2024-01-22)
------------------
* Update CHANGELOG.rst to move 1.0.0 to 0.0.0
* Contributors: David Dorf

0.0.4 (2024-01-16)
------------------

0.0.1 (2024-01-11)
------------------
* Update README.md with C++ version link
* Changed the Python readme instead of the C++ one.
* Add README.md
* Move publisher initialization to node initializer instead of inside function.
* PEP257 and Flake8 pass all tests.
* Pass flake tests, working on pep257.
* Update README.md
* Contributors: David Dorf, daviddorf2023

0.0.0 (2023-11-14)
------------------
* Renaming to meet REP144
* Update README.md with video
* Add installation to README.md
* [STABLE] Docstrings and minor adjustments.
* Bugfixes to sphere marker publishing in 3D. Separate RViz config files for 2D and 3D.
* Change obstacle_pub to obstacle_pub_2D for implementing a separate 3D version.
* Implemented the features added to 2D into rrt3D [besides map features since octomaps are shaky in ROS 2]. Deleted class docstrings from rrt2D and rrt3D in preparation for final documentation.
* Widescale improvements that allows obstacle pub to send markers over a topic to rrt2D obstacle subscriber.
* Add docstrings to set_start_goal
* Change Obstacles.py to Marker.py and move create_marker method to it.
* Use a loop to get params in 2D, obstacle publisher initialized
* Bugfix, map matrix had to be transposed the whole time (it was mirrored)
* Add logger info outputs for starting RRT, receiving map data, and completing the path.
* [STABLE] Cleanup and documentation.
* [STABLE] 2D case: Add timer callback so that path and nodes always are published. Real-time marker appending in RViz, so plotting no longer necessary. Goal and start marker are now larger.
* [STABLE] Uniform instead of randint for non-int step sizes
* [STABLE] Bugfixes, cleanup, use map resolution instead of step size for plotting, add wall confidence threshold
* rrt2D node avoids walls, shows markers for start/goal, has a scale based on step_size, waits for map data from subscriber, and plots walls/obstacles.
* Obtained wall pixel center coordinates and set up function to prevent start/goal from being inside a marker obstacle, wall pixel, or out of map bounds.
* Untested. Should prevent node generation if closer than the step size to a wall pixel.
* Subscriber to map in rrt2D gets map data.
* [STABLE] Added occupancy grid publisher to 2D rrt
* Update README.md with new videos
* Add docstrings, remove videos and images.
* [STABLE] Added path publisher to 2D case.
* [STABLE] 3D case publishes a path
* [STABLE] Collision avoidance in 2D and 3D. RViz markers for goal and start position. WIP: Publishing path to goal, user input obstacles, obstacle-goal/start collision check.
* [STABLE] Obstacles added into RViz. RRT doesn't go around them yet.
* Introduce obstacle classes
* [STABLE] 2D and 3D cases both work
* 2D RViz and separate 2D and 3D launch files
* Update README.md with video
* Initial commit. Transfers contents from my GitLab to this GitHub repo.
* Contributors: David Dorf, daviddorf2023
