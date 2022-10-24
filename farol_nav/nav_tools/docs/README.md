# Nav tools

## Project Description

This package contains tools to convert msgs between data types. Currently, it also houses range-based positioning tools. This package has the following highlights

- Convert data in WGS84 from *sensor_msgs::NavSatFix* to data in UTM NED & *farol_msgs::Measurement*
- Convert state from *auv_msgs::NavigationStatus* to *farol_msgs::mState*
- Convert state from *auv_msgs::NavigationStatus* to *nav_msgs::Odometry*
- Convert Georeferenced USBL position fix to position update

## Nodes
[auvstate2mstate](./AuvState2mState.md)
[gnss2utm](./gnss2utm.md)
[gnss2utmoutlier](./Gnss2UtmlOutlier.md)
[usbl2pos](./UsblFix2Pos.md)

## Package Content

