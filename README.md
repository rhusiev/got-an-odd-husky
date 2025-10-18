Clone `https://github.com/UCU-robotics-lab/turtlebot4_sim_ws`

Start: `./start.sh` or `./start.sh <path-to-turtlebot4_sim_ws>`

Then `docker exec -ti ros_dev bash`

In container run `./init.sh` and `./build.sh` from `/workspace/turtlebot4_sim_ws`, then run the scripts with the same names from `/workspace/got-an-odd-husky`

Re-run the latter `./build.sh` when changing the program

Lastly, `ros2 run got_an_odd_husky controller_node`

In the end `docker stop ros_dev` and `docker rm ros_dev`
