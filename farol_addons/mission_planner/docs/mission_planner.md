# Mission Planner Node

## Diagram
<!-- ![mission_planner Diagram](img/data_serializer.png) -->

## Subscribers

- "/#vehicle#/nav/filter/state" [auv_msgs/NavigationStatus]
- "/#vehicle#/mission_planner/interest_zone" [mission_planner/mInterestZone]
- "/#vehicle#/acomms/new_iz_mission" [mission_planner/mNewIZMission]
- "/#vehicle#/acomms/being_scanned" [std_msgs/Empty]
- "/#vehicle#/acomms/vehicle_ready" [std_msgs/Int8]
- "/#vehicle#/acomms/mission_started_ack" [mission_planner/mMissionStartedAck]

## Publishers

- "/#vehicle#/addons/Mission_String" [std_msgs/String]
- "/#vehicle#/mission_planner/mission_started_ack" [std_msgs/Int8]
- "/#vehicle#/mission_planner/new_iz_mission" [mission_planner/mNewIZMission]
- "/#vehicle#/mission_planner/ready_for_mission" [std_msgs/Int8]
- "/#vehicle#/mission_planner/stop_all_pf" [std_msgs::Empty]

## Services

- /#vehicle#/mission_planner/change_configurations [mission_planner/Configs]
- /#vehicle#/mission_planner/interest_zone [mission_planner/InterestZone]

## Parameters

- p_node_frequency_
- path_orientation_
- path_type_
- min_turning_radius_
- path_speed_
- resolution_
- dist_inter_vehicles_
- vehicle_id_
- timeout_ack_