/* Simple C++ client example.
 * Based on example0.cc from player distribution
 * K. Nickels 7/24/13
 */

#include <iostream>
#include <libplayerc++/playerc++.h>

int main(int argc, char *argv[]) {
  using namespace PlayerCc;
  using namespace std;

  PlayerClient    robot("localhost");
  Position2dProxy p2dProxy(&robot,0);
/*  RangerProxy     sonarProxy(&robot,0);
  RangerProxy     toothProxy(&robot,1);
  RangerProxy     laserProxy(&robot,2);
*/
  MapProxy          mapProxy(&robot,0);
  RangerProxy     laserProxy(&robot,2);

    // read from the proxies
    robot.Read();

/*    sonarProxy.RequestGeom();
    sonarProxy.RequestConfigure();

    toothProxy.RequestGeom();
    toothProxy.RequestConfigure();
*/
    laserProxy.RequestGeom();
    laserProxy.RequestConfigure();
/*
    cout << sonarProxy << endl;
    cout << toothProxy << endl;
    cout << laserProxy << endl;
*/
    robot.Read();

    // Method 1
/*
    cout << sonarProxy.GetRangeCount() << " sonar ranges1: ";
    for (int i=0;i<sonarProxy.GetRangeCount()-1;i++) 
        cout<< sonarProxy[i] << ", ";
    cout << sonarProxy[sonarProxy.GetRangeCount()-1] << endl;

    // Method 2, same
    cout << sonarProxy.GetRangeCount() << " sonar ranges2: ";
    for (int i=0;i<sonarProxy.GetRangeCount()-1;i++) 
        cout<< sonarProxy.GetRange(i) << ", ";
    cout << sonarProxy.GetRange(sonarProxy.GetRangeCount()-1) << endl;
*/
    mapProxy.RequestMap();
    int8_t img[640*480];
    mapProxy.GetMap(img);
    cout << laserProxy.GetRangeCount() << " laser ranges"<<endl;
    for (int i=0;i<laserProxy.GetRangeCount()-1;i++){
        cout<< laserProxy[i] << ", ";

    }
    cout << laserProxy[laserProxy.GetRangeCount()-1] << endl;

    int para = 0;
    int todosLivres = 1;
    double velX = 0.5;
    double velAng = 0.0;
    while(para != 1)
    {
	if(todosLivres == 1)
	{
	    velX = 0.3;
	    velAng = 0.0;
	}
	
	p2dProxy.SetCarlike(velX,dtor(velAng)); // XSpeed, Yaw (rad)	
        robot.Read();
	
	todosLivres = 1;

	for(int i=0;i<laserProxy.GetRangeCount()-1;i++) {
	    if(laserProxy[i] < 0.5){
		if(i < 80)
		{
		    //velX -= 0.03;
		    velAng = 20;
		}else if(i < 90)
		{
		    //velX -= 0.3;
		    velAng = 20;
		}else if(i < 100)
		{
		    //velX -= 0.3;
		    velAng = -20;
		}else{
		    //velX -= 0.03;
		    velAng = -20;
		}

		todosLivres = 0;
            }
        }
    }



    return 0;
}

