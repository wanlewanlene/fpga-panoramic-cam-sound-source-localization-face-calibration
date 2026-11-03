# cam_mic 整合工程构建脚本（CLI 复现 IDE 构建）
#
# 两个关键点：
#  1) DONE 专用脚释放（G11 作 UART 诊断输出）必须由 project_process_config.json
#     提供，该文件由 settings/ 副本自动就位 —— 与 cam_dvi 的做法一致。
#  2) 必须显式指定顶层。工程里存在若干未被例化的模块，V1.9.12 的综合器
#     会随机挑一个当顶层（实测挑中过 uart_merge / timing_check），
#     结果只编了个空壳却报"综合成功"—— 典型的假通过。
set PROJ [file dirname [file normalize [info script]]]
file mkdir $PROJ/impl
file copy -force $PROJ/settings/project_process_config.json $PROJ/impl/project_process_config.json
file copy -force $PROJ/settings/project_process_config.json $PROJ/impl/cam_mic_process_config.json
open_project $PROJ/cam_mic.gprj
set_option -top_module top
run all
exit
