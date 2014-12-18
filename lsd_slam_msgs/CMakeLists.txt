cmake_minimum_required(VERSION 2.8.3)
project(lsd_slam_msgs)

# Declare dependencies
set( MESSAGE_DEPENDENCIES
	nav_msgs
	sensor_msgs
	std_msgs
	trajectory_msgs
	visualization_msgs
)

# Find dependencies
find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	${MESSAGE_DEPENDENCIES} 
)

# Add message files
add_message_files(
	FILES
	keyframeGraphMsg.msg
	keyframeMsg.msg
)

# Add service files
# add_service_files(
#     FILES
# )

# generate messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

# Declare run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})
