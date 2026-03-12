set pagination off
set confirm off
file D:/myFiles/RM/uav_basic_framework/uav_baice_framework_v1.6/cmake-build-debug/uav_baice_framework_v1.6.elf
target remote 127.0.0.1:2331
monitor reset
load
monitor reset
set $bmi088_cb = 0
set $bmi270_cb = 0
set $spl06_cb = 0
set $qmc_cb = 0
break BMI088_DMA_Callback
commands
silent
set $bmi088_cb = $bmi088_cb + 1
continue
end
break BMI270_DMA_Callback
commands
silent
set $bmi270_cb = $bmi270_cb + 1
continue
end
break SPL06_Data_Handler
commands
silent
set $spl06_cb = $spl06_cb + 1
continue
end
break QMC5883L_DMA_Callback
commands
silent
set $qmc_cb = $qmc_cb + 1
if $qmc_cb < 10
  continue
end
end
continue
printf "BP_BMI088=%d\n", $bmi088_cb
printf "BP_BMI270=%d\n", $bmi270_cb
printf "BP_SPL06=%d\n", $spl06_cb
printf "BP_QMC=%d\n", $qmc_cb
print g_sensor_diag
print bmi270_init_done
quit
