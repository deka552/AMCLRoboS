# ------------------------------------------------------------------------------
#                                ALIASES
# ------------------------------------------------------------------------------

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)

DOCKER_COMPOSE_FILES := \
	-f $(ROOT_DIR)/docker-compose.yaml

# ------------------------------------------------------------------------------
#                                 IMAGES
# ------------------------------------------------------------------------------

IMAGES := uart lidar navigation
CARTOGRAPHER := cartographer
NAV := navigation
LIADR := lidar
UART := uart
GAZ := gazebo
SAVE_MAP := save_map


# ------------------------------------------------------------------------------
#                               PARAMETERS
# ------------------------------------------------------------------------------

# Configure the serial connection

SERIAL_PORT ?= /dev/ttyUSB0
BAUD_RATE ?= 115200
PACKET_FORMAT ?= '<3sBIdddddd'
PACKET_FORMAT_RETURN ?= '<3sBIdddd'

# Axes transform parameters

X_Y_FLIP ?= True			# True | False
X_DIRECTION ?= -1    		# 1 or -1
Y_DIRECTION ?= -1     		# 1 or -1
X_DELTA ?= 5.6        		# meters  # 2.0 	| 2.4
Y_DELTA ?= 2.4    		# meters  # 21.59 	| 5.6
ANGLE_DELTA ?= 0   		# in degrees 270 | 180

# ROS parameters

USE_RVIZ ?= True			# True | False

# Other parameters

SEND_LOCALIZATION ?= True
VALID_DELTA_X ?= 1.0
VALID_DELTA_Y ?= 1.0

# ------------------------------------------------------------------------------

PARAMETERS := \
	ROOT_DIR=$(ROOT_DIR) \
	SERIAL_PORT=$(SERIAL_PORT) \
	BAUD_RATE=$(BAUD_RATE) \
	PACKET_FORMAT=$(PACKET_FORMAT) \
	PACKET_FORMAT_RETURN=$(PACKET_FORMAT_RETURN) \
	X_Y_FLIP=$(X_Y_FLIP) \
	X_DIRECTION=$(X_DIRECTION) \
	Y_DIRECTION=$(Y_DIRECTION) \
	X_DELTA=$(X_DELTA) \
	Y_DELTA=$(Y_DELTA) \
	ANGLE_DELTA=$(ANGLE_DELTA) \
	USE_RVIZ=$(USE_RVIZ) \
	SEND_LOCALIZATION=$(SEND_LOCALIZATION) \
	VALID_DELTA_X=$(VALID_DELTA_X) \
	VALID_DELTA_Y=$(VALID_DELTA_Y)

# ------------------------------------------------------------------------------
#                                COMMANDS
# ------------------------------------------------------------------------------

BUILD_COMMAND := docker compose $(DOCKER_COMPOSE_FILES) build
RUN_COMMAND := docker compose $(DOCKER_COMPOSE_FILES) up

# ------------------------------------------------------------------------------
#                             GENERAL INTERFACE
# ------------------------------------------------------------------------------

build:
	@echo "ROOT_DIR: $(ROOT_DIR)"
	@echo "Building: $(IMAGES)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(BUILD_COMMAND) $(CARTOGRAPHER)

	
# ------------------------------------------------------------------------------

run_all: 
	xhost +local:docker
	@echo "Running: $(IMAGES)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(IMAGES)


run_cartographer: 
	xhost +local:docker
	@echo "Running: $(CARTOGRAPHER)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(CARTOGRAPHER)

run_navigation: 
	xhost +local:docker
	@echo "Running: $(NAV)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(NAV)

run_lidar: 
	xhost +local:docker
	@echo "Running: $(LIADR)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(LIADR)

run_uart: 
	@echo "Running: $(UART)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(UART)

run_gazebo: 
	xhost +local:docker
	@echo "Running: $(GAZ)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(GAZ)

save_map: 
	@echo "Running: $(SAVE_MAP)"
	cd $(ROOT_DIR) && $(PARAMETERS) $(RUN_COMMAND) $(SAVE_MAP)

