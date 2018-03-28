source "../devel/setup.bash"
echo "Loading calibration data and swarmie_control sketch"
./load_swarmie_control_sketch.sh $2
../src/sipi_controller/scripts/rover_onboard_node_launch.sh $1 $2
