/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h" 


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{

	ldata = laser_proxy->getLaserData();
	
	switch(state)
	{
		case State::INIT:
			//cambiar cuando se pulse el botón desde la UI
			break;
		
		case State::ADVANCE:
			search();
			break;
		
		case State::ADVANCE:
			navegar();
			break;
		
		
	}
	
	
}


void SpecificWorker::search()
{
			girar hasta tList.exists( currentTag );
			state = State::ADVANCE;
	
}



void navegar()
{
	const float threshold = 400;
    float rot = 1.5707,rot1; 



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+35, ldata.begin()+65, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;

		

     if( ldata[35].dist < (sqrt(pow(threshold,2)+pow(threshold,2))))
    {
		/*
		int i=rand()%2;
		rot1=rot;
		if(i==0)
			rot1=rot*(-1);*/
		if(ldata.front().dist<=ldata[99].dist){
			rot1=rot*(-1);
			std::cout << "Izquierda" << std::endl; 
			
		}
		else{
			
			rot1=rot;
			std::cout << "Derecha" << std::endl;
		}
    differentialrobot_proxy->setSpeedBase(5, rot1);
    usleep(1250000);
    std::cout << ldata.front().dist << std::endl;   
    differentialrobot_proxy->setSpeedBase(200, 0);
    usleep(500000);

    rot = rot + 0.12;
    if( rot > 3 * 1.5707 )
    {
     rot = 1.5707;
    }
    
    // tagList.get( id);
    }

    else
    {
    differentialrobot_proxy->setSpeedBase(200, 0); 
    usleep(500000);
    std::cout << ldata.front().dist << std::endl;
    }


    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}
////////////////////////////
////  ICE
/////////////////////////////


void SpecificWorker::newAprilTag(const tagsList& tags)
{
	for(auto t :tags)
	{
		qDebug()<<t.id;
		tList.add( t );
		
	}
	
}

