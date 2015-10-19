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
			marca=0;
			state = State::SEARCH;
			break;
		
		case State::SEARCH:
			search();
			break;
		
		case State::ORIENTATION:
			orientation();
			break;
		
		case State::ADVANCE:
			if(tList.exists( marca)){
				navegar();
			}
			else{
				state = State::SEARCH;
			}
			break;
			
		case State::FINISH:
			//navegar();
			break;
		
	}
	
	
}

void SpecificWorker::orientation()
{
	if((tList.get(marca).tx<=5) && (tList.get(marca).tx>=-5)){
		std::cout << "avanzo" << std::endl; 
			usleep(500000);
			state = State::ADVANCE;
	}
	else
		if(tList.get(marca).tx<0){
				differentialrobot_proxy->setSpeedBase(5, (-1.5707/12));
				usleep(100000);
				differentialrobot_proxy->setSpeedBase(0, 0);
		}
		else{
				differentialrobot_proxy->setSpeedBase(5, (1.5707/12));
				usleep(100000);
				differentialrobot_proxy->setSpeedBase(0, 0);
		}
			
}
void SpecificWorker::search()
{
			if(tList.exists( marca)){
				state = State::ORIENTATION;
			}
			//if(tList.map.begin().Tag.id<marca)
				differentialrobot_proxy->setSpeedBase(5, (1.5707/3));
			//else
			//	differentialrobot_proxy->setSpeedBase(5, (-1.5707/3));
			//girar hasta tList.exists( currentTag );
			//state = State::ADVANCE;
	
}



void SpecificWorker::navegar()
{
	const float threshold = 400;
    float rot = 1.5707,rot1; 



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+35, ldata.begin()+65, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;

		
			//std::cout << tList.distanceMark(marca) << std::endl;
	if(tList.distanceMark(marca)>(threshold+100)){
		if( ldata[35].dist < (sqrt(pow(threshold,2)+pow(threshold,2))))
		{
			if(ldata.front().dist<=ldata[99].dist)
			{
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
			differentialrobot_proxy->setSpeedBase(0, 0);
			usleep(1250000);
			differentialrobot_proxy->setSpeedBase(200, 0);
			if(!tList.exists(marca)){
				state = State::SEARCH;
			}
			usleep(500000);
    
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(200, 0); 
			usleep(500000);
			std::cout << ldata.front().dist << std::endl;
		}
	}
	else{
			if(tList.distanceMark(marca)>0){
				marca++;
				if(marca==4)
					state = State::FINISH;
				else
					state = State::SEARCH;
				differentialrobot_proxy->setSpeedBase(0, 0);
			} 
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
void SpecificWorker::vagabundear()
{
	const float threshold = 400;
    float rot = 1.5707,rot1; 



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+35, ldata.begin()+65, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;

    // tList.exists(marca);
	if(tList.distanceMark(marca)>threshold){
		if( ldata[35].dist < (sqrt(pow(threshold,2)+pow(threshold,2))))
		{
			if(ldata.front().dist<=ldata[99].dist)
			{
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
			//rot = rot + 0.12;
			//if( rot > 3 * 1.5707 )
			//{
			//	rot = 1.5707;
			//}
    
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(200, 0); 
			usleep(500000);
			std::cout << ldata.front().dist << std::endl;
		}
	}
	else{
			marca++;
			if(marca==4)
				state = State::FINISH;
			else
				state = State::SEARCH;
			differentialrobot_proxy->setSpeedBase(0, 0); 
	}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	for(auto t :tags)
	{
		qDebug()<<t.id;
		tList.add( t );
		
	}
	
}

