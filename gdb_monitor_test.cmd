set pagination off
set confirm off
file D:/myFiles/RM/uav_basic_framework/uav_baice_framework_v1.6/cmake-build-debug/uav_baice_framework_v1.6.elf
target remote 127.0.0.1:2331
monitor clrbp
monitor reset
load
monitor clrbp
monitor reset
monitor go
monitor sleep 20000
monitor halt
print g_sensor_diag
print bmi270_init_done
quit
