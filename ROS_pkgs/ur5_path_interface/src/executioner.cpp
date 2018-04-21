#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <ros/ros.h>

#include <string>
#include <stdexcept>


using namespace std;


class UR5path
{
 public:
  UR5path() : nodehandle_("~"), sdsip_(nodehandle_, "ur5_path_executing")
  {
    initWorkCell();
    initDevice();
    initPathPlannerWithCollisionDetector();
  }

  virtual ~UR5path()
  {
    /* Empty */
  }

  bool testMoveServoQ(const double q_change)
  {
    if (!doTestMoveServoQ(q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    if (!doTestMoveServoQ(-q_change))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    return true;
  }

 protected:
  void initWorkCell()
  {
    workcell_ = caros::getWorkCell("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml");
    if (workcell_ == NULL)
    {
      ROS_ERROR("No workcell was loaded - exiting...");
      throw std::runtime_error("Not able to obtain a workcell.");
    }
  }

//-------------------------------------------------------------------------------------------

  void initDevice()
  {
    std::string device_name;
    if (!nodehandle_.getParam("device_name", device_name))
    {
      ROS_FATAL_STREAM("The parameter '" << nodehandle_.getNamespace()
                                         << "/device_name' was not present on the parameter "
                                            "server! This parameter has to be specified "
                                            "for this test-node to work properly.");
      throw std::runtime_error("Not able to obtain device name.");
    }

    ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
    device_ = workcell_->findDevice(device_name);
    if (device_ == NULL)
    {
      ROS_FATAL_STREAM("Unable to find device " << device_name << " in the loaded workcell");
      throw std::runtime_error("Not able to find the device within the workcell.");
    }
  }

//--------------------------------------------------------------------------------------------

  void initPathPlannerWithCollisionDetector()
  {

    rw::kinematics::State state = workcell_->getDefaultState();
    /* Collision detector */
    auto detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(workcell_, rwlibs::proximitystrategies::ProximityStrategyPQP::make()));
    /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
     * collision free and that the edge(s) between those is/are also collision free. */
    const rw::pathplanning::PlannerConstraint planner_constraint =
        rw::pathplanning::PlannerConstraint::make(detector, device_, state);

    /* Just using a really simple path planner (straight line in the configuration space) */
   // planner_ = rw::pathplanning::QToQPlanner::make(planner_constraint);
      planner_ = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(planner_constraint, rw::pathplanning::QSampler::makeUniform(device_),rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), 0.1);

  } // initPathPlannerWithCollisionDetector()


//----------------------------------------------------------------------------------------------


  rw::math::Q getCurrentJointConfiguration()
  {
    /* Make sure to get and operate on fresh data from the serial device
     * It's assumed that the serial device is not moving
     * ^- That could be asserted / verified using sdsip.isMoving()
     * However other sources could invoke services on the UR that causes it to move...
     */
    ros::Time current_timestamp = ros::Time::now();
    ros::Time obtained_timestamp = sdsip_.getTimeStamp();
    while (current_timestamp > obtained_timestamp)
    {

      ros::Duration(0.1).sleep();  // In seconds
      ros::spinOnce();
      obtained_timestamp = sdsip_.getTimeStamp();

    } // while

    return sdsip_.getQ();

  } // getCurrentJointConfiguration()

//-----------------------------------------------------------------------------------------------

   /*

	 
		¡¡¡¡¡¡¡   AQUÍ LA NUEVA FUNCION DE INTERPOLACION DEL PATH   !!!!!!!

		It needs changes: no q_change  -------------------> Initial and final Q() + RRT planning + interpolation (¿Linear or cubic?)
		

   */  



 /*
  rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end, const double total_duration = 10.0, const double duration_step = 1.0)
  {
    ROS_ASSERT(duration_step > 0);
    ROS_ASSERT(duration_step < total_duration);

    rw::trajectory::QLinearInterpolator interpolator(start, end, total_duration);

    rw::trajectory::QPath path;

    path.push_back(start);
    for (double t = duration_step; t <= (total_duration - duration_step); t += duration_step)
    {
      path.push_back(interpolator.x(t));
    }
    path.push_back(end);

    return path;
  }*/

//-------------------------------------------------------------------------------------------------

  bool doTestMoveServoQ(const double q_change)
  {
    bool return_status = true;
    rw::trajectory::QPath path = getQPath(q_change);

    ROS_ASSERT(path.size() == 2);
    rw::math::Q start_configuration = path.at(0);
    rw::math::Q end_configuration = path.at(1);

    // ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡  AQUÍ LA NUEVA FUNCION DE OBTENER PATH   !!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // replace the path with an interpolated path
    //path = linearInterpolatedPath(start_configuration, end_configuration);




    for (const rw::math::Q& p : path)
    {
      ROS_INFO_STREAM("Ask to moveServoQ to '" << p << "'.");
      bool ret = false;
      ret = sdsip_.moveServoQ(p);
      if (!ret)
      {
        return_status = false;
        ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
      
      } // if
    
    } // for

    return return_status;

  } // doTestMoveServoQ()

//----------------------------------------------------------------------------------------------------

 protected:
  ros::NodeHandle nodehandle_;
  caros::SerialDeviceSIProxy sdsip_;

  rw::models::WorkCell::Ptr workcell_;
  rw::models::Device::Ptr device_;
  rw::pathplanning::QToQPlanner::Ptr planner_;

}; // class UR5path





// ==============================================================================================================================================================================================







int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_servo_q");

  const double q_change = 0.2;

  UR5path ur_path;
  bool ret = false;
  ret = ur_path.testMoveServoQ(q_change);



  if (!ret)
  {
    ROS_ERROR_STREAM("Could not properly do the testMoveServoQ");
    return 1;
  }



  return 0;

} // main
