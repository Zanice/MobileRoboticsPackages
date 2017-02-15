echo "Enter the eth0 inet address (use 'ifconfig'):"
read THIS_IP

echo "Enter the IP address of the ROS interface:"
read ROS_IP

echo "Using <eth0=$THIS_IP ros=$ROS_IP>"

echo ">> Running setup bash script..."
source /opt/ros/indigo/setup.bash
echo ">> Exporting master URI to ROS interface..."
export ROS_MASTER_URI=$ROS_IP:11311
echo ">> Exporting ROS IP to eth0 address..."
export ROS_IP=$THIS_IP

echo "Setup complete."

