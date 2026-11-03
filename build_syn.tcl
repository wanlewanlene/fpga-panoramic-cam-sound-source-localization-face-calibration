set PROJ [file dirname [file normalize [info script]]]
file mkdir $PROJ/impl
file copy -force $PROJ/settings/project_process_config.json $PROJ/impl/project_process_config.json
file copy -force $PROJ/settings/project_process_config.json $PROJ/impl/cam_mic_process_config.json
open_project $PROJ/cam_mic.gprj
set_option -top_module top
run syn
exit
