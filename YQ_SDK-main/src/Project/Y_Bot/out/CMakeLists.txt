# --------------------------
# 编译 Y_Bot_Out 可执行文件
# --------------------------
set(EXE_NAME Y_Bot_demo)
NAME_TO_VAR("PROJECT" "-${EXE_NAME}" OUT)
if(${OUT})
    # 添加可执行文件+源文件
    add_executable(${EXE_NAME} 
        ./Y_Bot_Out.cpp
    )
    
    target_include_directories(${EXE_NAME} PUBLIC
        ${YAML_CPP_INCLUDE_DIR}
        ${SYS_INC_DIR}
        ${INC_SUBDIRS} 
    )

    add_dependencies(${EXE_NAME}
        HARDWARE_TOP
        Switch_Board
        Switch_Board_Orin
        Motor_Device
        Auto_Set_Id
        Motor_Device_TaiHu
        Motor_Interpolation
        Pressure_Sensor_Custom_Device
    )

    config_debug_mode(${EXE_NAME})

    target_link_libraries(${EXE_NAME} PRIVATE
        HARDWARE_TOP
        Switch_Board
        Switch_Board_Orin
        Motor_Device
        Auto_Set_Id
        Motor_Interpolation
        Motor_Device_TaiHu
        Pressure_Sensor_Custom_Device

        pthread
        -Wl,--no-as-needed 
        ${YAML_CPP_LIB}      
        spdlog_global    
        -Wl,--as-needed      
        )
endif()


# --------------------------
# 编译 Y_Bot_Out_orin_fdcan 可执行文件
# --------------------------
set(EXE_NAME Y_Bot_demo_orin_fdcan)
NAME_TO_VAR("PROJECT" "-${EXE_NAME}" OUT)
if(${OUT})
    # 添加可执行文件+源文件
    add_executable(${EXE_NAME} 
        ./Y_Bot_Out_orin_fdcan.cpp
    )
    
    target_include_directories(${EXE_NAME} PUBLIC
        ${YAML_CPP_INCLUDE_DIR}
        ${SYS_INC_DIR}
        ${INC_SUBDIRS} 
    )

    add_dependencies(${EXE_NAME}
        HARDWARE_TOP
        Switch_Board
        Switch_Board_Orin
        Motor_Device
        Auto_Set_Id
        Pressure_Sensor_Custom_Device
    )

    config_debug_mode(${EXE_NAME})

    target_link_libraries(${EXE_NAME} PRIVATE
        HARDWARE_TOP
        Switch_Board
        Switch_Board_Orin
        Motor_Device
        Auto_Set_Id
        Pressure_Sensor_Custom_Device

        pthread
        -Wl,--no-as-needed 
        ${YAML_CPP_LIB}      
        spdlog_global    
        -Wl,--as-needed      
        )
endif()


# --------------------------
# 编译 Y_Bot_Out_fdcan 可执行文件
# --------------------------
set(EXE_NAME Y_Bot_demo_fdcan)
NAME_TO_VAR("PROJECT" "-${EXE_NAME}" OUT)
if(${OUT})
    # 添加可执行文件+源文件
    add_executable(${EXE_NAME} 
        ./Y_Bot_Out_fdcan.cpp
    )
    
    target_include_directories(${EXE_NAME} PUBLIC
        ${YAML_CPP_INCLUDE_DIR}
        ${SYS_INC_DIR}
        ${INC_SUBDIRS} 
    )

    add_dependencies(${EXE_NAME}
        HARDWARE_TOP
        Switch_Board
        Motor_Device
        Auto_Set_Id
        Pressure_Sensor_Custom_Device
    )

    config_debug_mode(${EXE_NAME})

    target_link_libraries(${EXE_NAME} PRIVATE
        HARDWARE_TOP
        Switch_Board
        Motor_Device
        Auto_Set_Id
        Pressure_Sensor_Custom_Device

        pthread
        -Wl,--no-as-needed 
        ${YAML_CPP_LIB}      
        spdlog_global    
        -Wl,--as-needed      
        )
endif()

Config_Mod_Dir_Cmake()


