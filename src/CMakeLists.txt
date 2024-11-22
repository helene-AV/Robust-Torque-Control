set(controller_SRC
  RobustTorqueControl.cpp
)

set(controller_HDR
  RobustTorqueControl.h
)



add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DRobustTorqueControl_EXPORTS")

