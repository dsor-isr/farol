/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "DmacToFarolNode"
#include "DmacToFarolNode.h"

/*
@.@ CONSTRUCTOR: put all dirty work of initializations here
*/
DmacToFarolNode::DmacToFarolNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
    ROS_INFO("in class constructor of DmacToFarolNode");
    loadParams();
    buildUSBLRotationMatrix();
    initializeSubscribers();
    initializePublishers();
}

/*
@.@ Destructor
*/
DmacToFarolNode::~DmacToFarolNode()
{

    // shutdown publishers
    usbl_fix_farol_pub_.shutdown();

    // shutdown subscribers
    usbl_fix_dmac_sub_.shutdown();

    // shutdown node
    nh_.shutdown();
    nh_private_.shutdown();
}

/*
 @.@ Member Helper function to set up subscribers;
 */
void DmacToFarolNode::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers for DmacToFarolNode");
    // Topic in
    p_usbl_fix_dmac_topic_ = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/dmac_fix");
    usbl_fix_dmac_sub_ = nh_.subscribe(p_usbl_fix_dmac_topic_, 10, &DmacToFarolNode::fixCallback, this);
    state_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "/nav/filter/state"), 1, &DmacToFarolNode::stateCallback, this);
}

/*
 @.@ Member helper function to set up publishers;
 */
void DmacToFarolNode::initializePublishers()
{
    ROS_INFO("Initializing Publishers for DmacToFarolNode"); 

    // Topic out
    p_usbl_fix_farol_topic_ = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/farol_fix");
    usbl_fix_farol_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(p_usbl_fix_farol_topic_, 1);
    usbl_range_farol_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/farol_range"), 1);
    usbl_azimuth_farol_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/farol_azimuth"), 1);
}

/*
 @.@ Load the parameters
 */
void DmacToFarolNode::loadParams()
{
    ROS_INFO("Load the DmacToFarolNode parameters");

    p_installation_matrix_ = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "installation_matrix");

    usbl_has_AHRS_ = FarolGimmicks::getParameters<bool>(nh_private_, "usbl_has_AHRS", false);

}

/*
@.@ Helper method for rotating message from sensor frame to base_link(emo) or base_pose(real) frames
*/
void DmacToFarolNode::buildUSBLRotationMatrix(){

// transducer to body frame rotation
modem_to_body_rot_matrix_ << cos(p_installation_matrix_[2])*cos(p_installation_matrix_[1]), 
                                 cos(p_installation_matrix_[2])*sin(p_installation_matrix_[1])*sin((p_installation_matrix_[0])) - sin(p_installation_matrix_[2])*cos((p_installation_matrix_[0])),
                                     cos(p_installation_matrix_[2])*sin(p_installation_matrix_[1])*cos((p_installation_matrix_[0])) + sin(p_installation_matrix_[2])*sin((p_installation_matrix_[0])),
                             sin(p_installation_matrix_[2])*cos(p_installation_matrix_[1]), 
                                 sin(p_installation_matrix_[2])*sin(p_installation_matrix_[1])*sin((p_installation_matrix_[0])) + cos(p_installation_matrix_[2])*cos((p_installation_matrix_[0])), 
                                     sin(p_installation_matrix_[2])*sin(p_installation_matrix_[1])*cos((p_installation_matrix_[0])) - cos(p_installation_matrix_[2])*sin((p_installation_matrix_[0])),
                             -sin(p_installation_matrix_[1]), 
                                 cos(p_installation_matrix_[1])*sin((p_installation_matrix_[0])), 
                                     cos(p_installation_matrix_[1])*cos((p_installation_matrix_[0]));

}

/*
@.@ Callback fix usbl dmac -> convert usbl fix dmac message into usbl fix farol dmac message
*/
void DmacToFarolNode::fixCallback(const dmac::mUSBLFix &dmac_usbl_fix_msg)
{
    // Start building the farol usbl fix message
    farol_msgs::mUSBLFix farol_usbl_fix_msg;

    farol_usbl_fix_msg.header = dmac_usbl_fix_msg.header;
    farol_usbl_fix_msg.type = dmac_usbl_fix_msg.type;

    farol_usbl_fix_msg.source_id = dmac_usbl_fix_msg.source_id;
    farol_usbl_fix_msg.source_name = dmac_usbl_fix_msg.source_name;
    farol_usbl_fix_msg.source_frame_id = "vehicle" + dmac_usbl_fix_msg.source_name;

    farol_usbl_fix_msg.bearing_raw = dmac_usbl_fix_msg.bearing_raw;
    farol_usbl_fix_msg.elevation_raw = dmac_usbl_fix_msg.elevation_raw;
    

    // If message is range only or full fix grab the range value from dmac message
    if (farol_usbl_fix_msg.type == farol_usbl_fix_msg.RANGE_ONLY || farol_usbl_fix_msg.type == farol_usbl_fix_msg.FULL_FIX)
    {
        farol_usbl_fix_msg.range = dmac_usbl_fix_msg.range;
        usbl_range_farol_pub_.publish(farol_usbl_fix_msg);
    }

    // If message is
    if (farol_usbl_fix_msg.type == farol_usbl_fix_msg.AZIMUTH_ONLY || farol_usbl_fix_msg.type == farol_usbl_fix_msg.FULL_FIX)
    {   
      if (!usbl_has_AHRS_){
        // ROS_WARN("[MODEM] BEARING, ELEVATION: %f, %f", dmac_usbl_fix_msg.bearing_raw, dmac_usbl_fix_msg.elevation_raw);

        // convert local bearing and elevation to a unit vector in 3D space (polar to cartesian coordinates)
        double x_rel, y_rel, z_rel;
        x_rel = cos(dmac_usbl_fix_msg.bearing_raw)*cos(dmac_usbl_fix_msg.elevation_raw);
        y_rel = sin(dmac_usbl_fix_msg.bearing_raw)*cos(dmac_usbl_fix_msg.elevation_raw);
        z_rel = sin(dmac_usbl_fix_msg.elevation_raw);

        // ROS_WARN("[MODEM] POINT: %f %f %f", x_rel, y_rel, z_rel);

        // rotate unit vector to body frame
        Eigen::MatrixXd point(3,1);
        point << x_rel, y_rel, z_rel;
        Eigen::MatrixXd pt_body;
        pt_body = modem_to_body_rot_matrix_*point;

        // ROS_WARN("[BODY] POINT: %f %f %f", pt_body(0), pt_body(1), pt_body(2));
      
      
        // Rotate unit vector from body to inertial frame
        // body_rot_matrix_ << cos(yaw_state_), -sin(yaw_state_), 0,
        //                    sin(yaw_state_), cos(yaw_state_), 0,
        //                    0,0,1;

        // body to inertial frame rotation
        body_to_inertial_rot_matrix_ << cos(yaw_state_)*cos(pitch_state_), 
                                            cos(yaw_state_)*sin(pitch_state_)*sin(roll_state_) - sin(yaw_state_)*cos(roll_state_),
                                                cos(yaw_state_)*sin(pitch_state_)*cos(roll_state_) + sin(yaw_state_)*sin(roll_state_),
                                        sin(yaw_state_)*cos(pitch_state_), 
                                            sin(yaw_state_)*sin(pitch_state_)*sin(roll_state_) + cos(yaw_state_)*cos(roll_state_), 
                                                sin(yaw_state_)*sin(pitch_state_)*cos(roll_state_) - cos(yaw_state_)*sin(roll_state_),
                                        -sin(pitch_state_), 
                                            cos(pitch_state_)*sin(roll_state_), 
                                                cos(pitch_state_)*cos(roll_state_);

        Eigen::MatrixXd pt_inertial = body_to_inertial_rot_matrix_ * pt_body;

        // ROS_WARN("[INERTIAL] POINT: %f %f %f", pt_inertial(0), pt_inertial(1), pt_inertial(2));

        // cartesian to sphere
        farol_usbl_fix_msg.bearing  = std::atan2(pt_inertial(1),pt_inertial(0));
        farol_usbl_fix_msg.elevation = std::atan2(pt_inertial(2),std::sqrt(std::pow(pt_inertial(0),2) + std::pow(pt_inertial(1),2)));
        
        // ROS_WARN("[INERTIAL] BEARING, ELEVATION: %f, %f", farol_usbl_fix_msg.bearing, farol_usbl_fix_msg.elevation);

      }
      else{
        farol_usbl_fix_msg.bearing = dmac_usbl_fix_msg.bearing;
        farol_usbl_fix_msg.elevation = dmac_usbl_fix_msg.elevation;
      }
      usbl_azimuth_farol_pub_.publish(farol_usbl_fix_msg);
    }
    // Publishing the new message farol usbl fix message
    usbl_fix_farol_pub_.publish(farol_usbl_fix_msg);
}


void DmacToFarolNode::stateCallback(const auv_msgs::NavigationStatus &msg){
    yaw_state_ = (msg.orientation.z/180) * FarolGimmicks::PI;
    pitch_state_ = (msg.orientation.y/180) * FarolGimmicks::PI;
    roll_state_ = (msg.orientation.x/180) * FarolGimmicks::PI;
}
/*
 @.@ Main
 */
int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "acoustic_converters_node"); //node name
    // create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh, nh_p("~");

    ROS_INFO("main: instantiating an object of type DmacToFarolNode");

    // instantiate an DmacToFarolNode class object and pass in pointer to nodehandle for constructor to use
    DmacToFarolNode DmacToFarol(&nh, &nh_p);

    // Added to work with timer -> going into spin; let the callbacks do all the work
    ros::spin();

    return 0;
}
