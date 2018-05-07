#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP


#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include "/home/charlie/catkin_ws/build/robot_plugin/ui_RobotPlugin.h"
#include "qtros.h"
#include <ros/ros.h>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <AnytimePlanning.h>

// Initialize plugin using QT5
class RobotPlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
        RobotPlugin();
        virtual ~RobotPlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();
	
	AnytimePlanning plan;

private slots:
	void btnPressed();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);
        void newState(rw::math::Q pos);

signals:
	void quitNow();
        void moveHome();

private:

	QTimer* _timer;
        QtROS *_qtRos;
	
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
        rw::models::Device::Ptr _device;



};

#endif /*RINGONHOOKPLUGIN_HPP_*/
