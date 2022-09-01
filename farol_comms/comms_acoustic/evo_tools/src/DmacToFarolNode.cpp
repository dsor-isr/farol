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

    // +.+ shutdown publishers
    usbl_fix_farol_pub_.shutdown();

    // +.+ shutdown subscribers
    usbl_fix_dmac_sub_.shutdown();

    // +.+ shutdown node
    nh_.shutdown();
    nh_private_.shutdown();
}

/*
 @.@ Member Helper function to set up subscribers;
 */
void DmacToFarolNode::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers for DmacToFarolNode");
    // +.+ Topic in
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

    // +.+ Topic out
    p_usbl_fix_farol_topic_ = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/farol_fix");
    usbl_fix_farol_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(p_usbl_fix_farol_topic_, 1);
}

/*
 @.@ Load the parameters
 */
void DmacToFarolNode::loadParams()
{
    ROS_INFO("Load the DmacToFarolNode parameters");

    p_real_ = FarolGimmicks::getParameters<bool>(nh_private_, "real", false);
    
    p_fix_type_ = FarolGimmicks::getParameters<bool>(nh_private_, "fix_type", false);

    p_installation_matrix_ = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "installation_matrix");

}

/*
@.@ Helper method for rotating message from sensor frame to base_link(emo) or base_pose(real) frames
*/
void DmacToFarolNode::buildUSBLRotationMatrix(){

  if (p_fix_type_ == false){
  usbl_rot_matrix_ << std::sin(p_installation_matrix_[0]), std::cos(p_installation_matrix_[0]), 0,
                      std::cos(p_installation_matrix_[0]), -std::sin(p_installation_matrix_[0]), 0,
                      0, 0, std::cos(p_installation_matrix_[2]);
}
else{
  usbl_rot_matrix_ << std::sin(p_installation_matrix_[0]), -std::cos(p_installation_matrix_[0]), 0,
                      std::cos(p_installation_matrix_[0]), std::sin(p_installation_matrix_[0]), 0,
                      0, 0, std::cos(p_installation_matrix_[2]);
}
}

/*
@.@ Callback fix usbl dmac -> convert usbl fix dmac message into usbl fix farol dmac message
*/
void DmacToFarolNode::fixCallback(const dmac::mUSBLFix &dmac_usbl_fix_msg)
{
    // +.+ Start building the farol usbl fix message
    farol_msgs::mUSBLFix farol_usbl_fix_msg;

    farol_usbl_fix_msg.header = dmac_usbl_fix_msg.header;
    farol_usbl_fix_msg.type = dmac_usbl_fix_msg.type;

    farol_usbl_fix_msg.source_id = dmac_usbl_fix_msg.source_id;
    farol_usbl_fix_msg.source_name = dmac_usbl_fix_msg.source_name;
    farol_usbl_fix_msg.source_frame_id = "vehicle" + dmac_usbl_fix_msg.source_name;

    farol_usbl_fix_msg.bearing_raw = dmac_usbl_fix_msg.bearing_raw;
    farol_usbl_fix_msg.elevation_raw = dmac_usbl_fix_msg.elevation_raw;

    // +.+ If message is range only or full fix grab the range value from dmac message
    if (farol_usbl_fix_msg.type == farol_usbl_fix_msg.RANGE_ONLY || farol_usbl_fix_msg.type == farol_usbl_fix_msg.FULL_FIX)
    {
        farol_usbl_fix_msg.range = dmac_usbl_fix_msg.range;
    }

    // If message is
    if (farol_usbl_fix_msg.type == farol_usbl_fix_msg.AZIMUTH_ONLY || farol_usbl_fix_msg.type == farol_usbl_fix_msg.FULL_FIX)
    {   
        
      if (p_real_){
        // sphere to cartesian usbl, assuming a range
        double x_rel, y_rel, z_rel, range_r=10.0;
        x_rel = cos(dmac_usbl_fix_msg.bearing_raw)*range_r*cos(dmac_usbl_fix_msg.elevation_raw);
        y_rel = sin(dmac_usbl_fix_msg.bearing_raw)*range_r*cos(dmac_usbl_fix_msg.elevation_raw);
        z_rel = range_r*sin(dmac_usbl_fix_msg.elevation_raw);

        // Rotate Vector to body
        Eigen::MatrixXd point(3,1);
        point << x_rel, y_rel, z_rel;
        Eigen::MatrixXd pt_rot;
        pt_rot = usbl_rot_matrix_*point;
      
      
        // Rotate Vector to interial
        body_rot_matrix_ << cos(yaw_state_), -sin(yaw_state_), 0,
                           sin(yaw_state_), cos(yaw_state_), 0,
                           0,0,1;
      
        pt_rot = body_rot_matrix_ * pt_rot;

        // cartesian to sphere
       farol_usbl_fix_msg.bearing  = std::atan2(pt_rot(1),pt_rot(0));
       farol_usbl_fix_msg.elevation = std::atan2(pt_rot(2),std::sqrt(std::pow(pt_rot(0),2) + std::pow(pt_rot(1),2)));

      }
      else{
        farol_usbl_fix_msg.bearing = dmac_usbl_fix_msg.bearing_raw;
        farol_usbl_fix_msg.elevation = dmac_usbl_fix_msg.elevation_raw;
      }
    }
    // +.+ Publishing the new message farol usbl fix message
    usbl_fix_farol_pub_.publish(farol_usbl_fix_msg);
}


void DmacToFarolNode::stateCallback(const auv_msgs::NavigationStatus &msg){
    yaw_state_ = (msg.orientation.z/180) * FarolGimmicks::PI;
}
/*
 @.@ Main
 */
int main(int argc, char **argv)
{
    // +.+ ROS set-ups:
    ros::init(argc, argv, "acoustic_converters_node"); //node name
    // +.+ create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh, nh_p("~");

    ROS_INFO("main: instantiating an object of type DmacToFarolNode");

    // +.+ instantiate an DmacToFarolNode class object and pass in pointer to nodehandle for constructor to use
    DmacToFarolNode DmacToFarol(&nh, &nh_p);

    // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
    ros::spin();

    return 0;
}
