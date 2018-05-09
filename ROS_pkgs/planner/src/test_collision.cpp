#include <AnytimePlanning.h>


using namespace std;


int main(int argc, char** argv) 
{

	AnytimePlanning plan;

	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2_with_ball/WC2_Scene.wc.xml";
	const string dev = "UR1";
	
	plan.Load_WorkCell(wcFile, dev);

	Object::Ptr obstacle = plan.wc->findObject("RedBall");
	
	if(obstacle == NULL)
		{cout << "Associated rigid object not found." << endl;}

	rw::math::Q q = rw::math::Q(6, -0.087, -1.007, 0.0, -1.573, 0.0, 0.0);
	plan.device->setQ(q, plan.state);

	cout <<"	>> CurrentQ = " << plan.device->getQ(plan.state) << endl;

	CollisionStrategy::Ptr S1 = plan.sphere_strategy(plan.state);
	CollisionDetector detector(plan.wc, S1);

	bool colliding;

	// Check collision		
	colliding = plan.checkCollisions(plan.state, detector, q);
	if (colliding == true)
	{
		cout << "	>> Collision detected in Q = " << q << endl;
		
	}// if
	else
	{
		cout << "	>> No collision" << endl;

	}// else


	return 0;

} // main()
