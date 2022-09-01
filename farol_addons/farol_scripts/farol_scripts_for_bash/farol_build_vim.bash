#!/bin/bash

catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1
bash ${FAROL_SCRIPTS}/farol_scripts_for_bash/concatenate_compile_commands.sh
ln -s build/compile_commands.json
cd $OLDPWD
