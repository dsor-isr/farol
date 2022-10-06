# Malaclemys personal alias

# FAROL SSH connections

alias myellow='ssh medusa@myellow'          # Myellow
alias delfim='ssh delfim@delfim'              # Delfim
alias muned='ssh medusa@muned'              # Muned
alias mblack='ssh medusa@mblack'            # Mblack
alias mred='ssh medusa@mred'                # Mred
alias hrov_pts_otg='ssh oceantech@hrov_otg' # Hrov port side
alias mvector='ssh medusa@mvector'	        # Mvector

# *.* FAROL Safety Features
alias sf_myellow='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client myellow'
alias sf_mvector='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client mvector'
alias sf_mred='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client  mred'
alias sf_mblack='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client mblack'
alias sf_delfim='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client  delfim'
alias sf_muned='$(find ${ROS_WORKSPACE}/src/ -type d -iname watchdog)/src/WatchdogClient/watchdog_client muned'
