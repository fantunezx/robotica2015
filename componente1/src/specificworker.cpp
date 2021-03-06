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
	tList.inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	tList.marca=0;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	timer.start(500);

	return true;
}

void SpecificWorker::compute()
{

	ldata = laser_proxy->getLaserData();
	TBaseState tbase;
	NavState estadocontrol;
	differentialrobot_proxy->getBaseState(tbase);
	tList.inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
	TargetPose t=tList.gettag();
	//qDebug() << "erghregas";
	//std::cout << "target: " << t.x << ", "  << t.y<< ", " << t.z << ".\n " << std::endl;
	//controller_proxy->go(t);
// 	switch(state)
// 	{
// 		case State::INIT:
// 			//cambiar cuando se pulse el botón desde la UI
// 			tList.marca=0;
// 			//usleep(100000);
// 			state = State::SEARCH;
// 			break;
// 		
// 		case State::SEARCH:
// 			search();
// 			break;
// 		
// 		case State::ORIENTATION:
// 			orientation();
// 			break;
// 		
// 		case State::ADVANCE:
// 			if(tList.exists( tList.marca)){
// 				navegar();
// 			}
// 			else{
// 				state = State::SEARCH;
// 				//tirar de memoria
// 			}
// 			break;
// 			
// 		
// 		case State::CONTROLLER:
// 			estadocontrol=controller_proxy->getState();
// 			if(estadocontrol.state == "IDLE")
// 			{
// 				std::cout << "target: " << t.x << ", "  << t.y<< ", " << t.z << ".\n " << std::endl;
// 				controller_proxy->go(t);
// 				espera=false;
// 			}
// 			if(estadocontrol.state=="FINISH"){
// 				if(espera==false){
// 					tList.marca++;
// 					espera=true;
// 					if(tList.marca==4)
// 						state = State::FINISH;
// 					else
// 						state = State::SEARCH;
// 					//controller_proxy->stop();
// 				}
// 			}
// 			break;
// 		case State::FINISH:
// 			//navegar();
// 			qDebug()<<"TERMINE";
// 			break;
// 		
// 	}
	
	
}

void SpecificWorker::orientation()
{
	float i;
	i=tList.distanceMark()-tList.get(tList.marca).tz;
	
	if((i<=5) && (i>=-5)){
		std::cout << "avanzo" << std::endl; 
			differentialrobot_proxy->setSpeedBase(0, 0);
			//usleep(500000);
			state = State::ADVANCE;
	}
	else
		if(tList.get(tList.marca).tx<0){
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
			if(tList.exists( tList.marca))
			{
				controller_proxy->stop();
				std::cout << "encontrado." << std::endl; 
				state = State::CONTROLLER;
			}else
				differentialrobot_proxy->setSpeedBase(5, (1.5707/3));
			
	
}



void SpecificWorker::navegar()
{
	const float threshold = 400;
    float rot = 1.5707,rot1; 



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+35, ldata.begin()+65, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;

			std::cout<<"Marca actual"<<tList.marca<<std::endl;
			std::cout << tList.distanceMark() << std::endl;
		
	if(tList.distanceMark()>(threshold+200))
	{ //si devuelve -1 no hacer
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
			usleep(1000000);
			std::cout << ldata.front().dist << std::endl;
			differentialrobot_proxy->setSpeedBase(0, 0);
			usleep(1000000);
			
			differentialrobot_proxy->setSpeedBase(200, 0);
			if(!tList.exists(tList.marca)){
				state = State::SEARCH;
			}
			usleep(2000000);
    
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(200, 0); 
			usleep(500000);
			std::cout << ldata.front().dist << std::endl;
		}
	}
	else{
			//if(tList.distanceMark(tList.marca)>0){
				tList.marca++;
				std::cout<<"Marca actual"<<tList.marca;
				if(tList.marca==4)
					state = State::FINISH;
				else
					state = State::SEARCH;
				differentialrobot_proxy->setSpeedBase(0, 0);
			//} 
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
	if(tList.distanceMark()>threshold){
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
			usleep(1000000);
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
			tList.marca++;
			if(tList.marca==4)
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

void SpecificWorker::reubicarse(){
  
  QVec t,ti,tib,tag,mem,valores,tbase;

  Apriltag_id= tList.inner->newTransform ("April_id", "static", tList.inner->getNode("rgbd"), 0, 0, 0, 0, 0, 0,0);
  
  switch(tList.marca){
    case 0:
      mem =tList.inner->transform6D("rgbd","target00"); 
      tList.inner->updateTransformValues("April_id",mem.x(),mem.y(),mem.z(),mem.rx(),mem.ry(),mem.rz(),"rgbd");
      ti = tList.inner->transform("April_id",QVec::zeros(6),"rgbd");
      rgbd_id= tList.inner->newTransform ("rgbd_id", "static", tList.inner->getNode("target00"), ti.x(), ti.y(), ti.z(), ti.rx(), ti.ry(), ti.rz(),0);
      break;
    case 1:
      mem =tList.inner->transform6D("rgbd","target01"); 
      tList.inner->updateTransformValues("April_id",mem.x(),mem.y(),mem.z(),mem.rx(),mem.ry(),mem.rz(),"rgbd");
      ti = tList.inner->transform("April_id",QVec::zeros(6),"rgbd");
      rgbd_id= tList.inner->newTransform ("rgbd_id", "static", tList.inner->getNode("target01"), ti.x(), ti.y(), ti.z(), ti.rx(), ti.ry(), ti.rz(),0);
      break;
    case 2:
      mem =tList.inner->transform6D("rgbd","target02"); 
      tList.inner->updateTransformValues("April_id",mem.x(),mem.y(),mem.z(),mem.rx(),mem.ry(),mem.rz(),"rgbd");
      ti = tList.inner->transform("April_id",QVec::zeros(6),"rgbd");
      rgbd_id= tList.inner->newTransform ("rgbd_id", "static", tList.inner->getNode("target02"), ti.x(), ti.y(), ti.z(), ti.rx(), ti.ry(), ti.rz(),0);
      break;
    case 3:
      mem =tList.inner->transform6D("rgbd","target03"); 
      tList.inner->updateTransformValues("April_id",mem.x(),mem.y(),mem.z(),mem.rx(),mem.ry(),mem.rz(),"rgbd");
      ti = tList.inner->transform("April_id",QVec::zeros(6),"rgbd");
      rgbd_id= tList.inner->newTransform ("rgbd_id", "static", tList.inner->getNode("target03"), ti.x(), ti.y(), ti.z(), ti.rx(), ti.ry(), ti.rz(),0);
      break;
  }
  
  tib= tList.inner->transform("base", QVec::zeros(6),"rgbd");
  base_id= tList.inner->newTransform ("base_id", "static", tList.inner->getNode("rgbd_id"), tib.x(), tib.y(), tib.z(), tib.rx(), tib.ry(), tib.rz(),0);
  valores = tList.inner->transform("world",QVec::zeros(6),"base_id");
  valores.print("valores");
  tList.inner->updateTransformValues("base",valores.x(),valores.y(),valores.z(),valores.rx(),valores.ry(),valores.rz());
  tbase= tList.inner->transform("world", QVec::zeros(6),"base");
  tbase.print("tbase");
  
  tList.inner->removeNode("April_id");
  tList.inner->removeNode("rgbd_id");
  tList.inner->removeNode("base_id");

  //*/
  //qFatal("fary");
}

/////////////////////////////////
/////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	for(auto t :tags)
	{
		//qDebug()<<t.id;
		tList.add( t);
		//if(tList.exists( tList.marca))
		//{
		//TagsList::Tag t=tList.get(tList.marca);
				
		//}
		
	}
	reubicarse();
	
}

