/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include <ros/ros.h>
#include <QThread>
#include <QObject>
#include <rw/math/Q.hpp>
#include <caros_control_msgs/RobotState.h>
#include <caros/serial_device_si_proxy.h>


class QtROS : public QThread {
  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtROS();

    /// This method contains the ROS event loop. Feel free to modify 
    void run();

  public slots:

    ///Connect to aboutToQuit signals, to stop the thread
    void quitNow();
    void moveHome();


  signals:

    ///Triggered if ros::ok() != true
    void rosQuits();

    /// Signal to emit for new configuration
    void newState(rw::math::Q);
  private:

    /// Callback function
    void stateCallback(const caros_control_msgs::RobotState & msg);

    bool quitfromgui;

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    caros::SerialDeviceSIProxy* _robot;


};
#endif

