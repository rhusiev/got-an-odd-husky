Clone `https://github.com/UCU-robotics-lab/turtlebot4_sim_ws`

Start: `./start.sh` or `./start.sh <path-to-turtlebot4_sim_ws>`

Then `docker exec -ti ros_dev bash`

After starting the container for the first time run `./init.sh` and `./build.sh` from `/workspace/turtlebot4_sim_ws`, then run `./init.sh` and `./build.sh` from `/workspace/got-an-odd-husky`

Relaunch the shell

Re-run the latter `./build.sh` when changing the program

Lastly, `ros2 run got_an_odd_husky controller_node`

In the end `docker stop ros_dev` and `docker rm ros_dev`
