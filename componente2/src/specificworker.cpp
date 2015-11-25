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
#include <math.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	estado.state= "IDLE";
	/*graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);*/
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
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
	ldata = laser_proxy->getLaserData();
	TBaseState tbase;
	differentialrobot_proxy->getBaseState(tbase);
	inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
    //std::sort( ldata.begin()+40, ldata.begin()+60, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	switch(state)
	{
		//No hace nada solo espera al go()
		case State::IDLE:
			break;
		//Estado inicial
		case State::INIT:
			qDebug()<<"Comienza la FIESTA!!!";
			estado.state= "INIT";
			coor.sub=false;
			state = State::GOAL;
			break;
		//Comprueba si ha llegado a donde queremos llegar
		case State::GOAL:
			goal();
			break;
		//Comprueba si el camino esta libre
		case State::FREEWAY:
			//buscar angulo y 
			freeway();
			break;
		//obtiene si el subtarget
		case State::NEWWAY:
			newway();
			break;
		//avanza hacia la marca
		case State::ADVANCE:
			advance();
			
			break;
		//No hace nada solo es para informar al de que ha terminado
		case State::FINISH:
			state = State::IDLE;
			estado.state= "FINISH";
			usleep(500000);
			estado.state= "IDLE";
			break;
		
	}
	//histogram();
}


void SpecificWorker::histogram()
{
static QGraphicsPolygonItem *p;
	static QGraphicsLineItem *l, *sr, *sl, *safety, *tline;
	const float R = 500; //Robot radius
	const float SAFETY = 500;

	scene.removeItem(p);
	scene.removeItem(l);
	scene.removeItem(sr);
	scene.removeItem(sl);
	scene.removeItem(safety);
	//scene.removeItem(tline);
	
	
	RoboCompLaser::TLaserData ldataI(ldata.size());
	for(int i=0; i<ldata.size(); i++)
		ldataI[ldata.size()-i-1]=ldata[i];
	//Search the first increasing step from the center to the right
	uint i,j;
	for(i=ldataI.size()/2; i>0; i--)
	{
		if( (ldataI[i].dist - ldataI[i-1].dist) < -R )
		{
			uint k=i-2;
			while( (k >= 0) and (fabs( ldataI[k].dist*sin(ldataI[k].angle - ldataI[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=ldataI.size()/2; j<ldataI.size()-1; j++)
	{
		if( (ldataI[j].dist - ldataI[j+1].dist) < -R )
		{
			uint k=j+2;
			while( (k < ldataI.size()) and (fabs( ldataI[k].dist*sin(ldataI[k].angle - ldataI[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	safety = scene.addLine(QLine(QPoint(0,-SAFETY/100),QPoint(ldata.size(),-SAFETY/100)), QPen(QColor(Qt::yellow)));
	sr = scene.addLine(QLine(QPoint(i,0),QPoint(i,-40)), QPen(QColor(Qt::blue)));
	sl = scene.addLine(QLine(QPoint(j,0),QPoint(j,-40)), QPen(QColor(Qt::magenta)));
	
	//tline = scene.addLine(QLine(QPoint(j,0),QPoint(j,-40)), QPen(QColor(Qt::brown)));
	
	//DRAW		
	QPolygonF poly;
	int x=0;
	poly << QPointF(0, 0);
	
	for(auto l : ldataI)
		poly << QPointF(x++, -l.dist/100); // << QPointF(x+5, d.dist) << QPointF(x+5, 0);
	poly << QPointF(x, 0);

	l = scene.addLine(QLine(QPoint(ldata.size()/2,0),QPoint(ldata.size()/2,-20)), QPen(QColor(Qt::red)));
    p = scene.addPolygon(poly, QPen(QColor(Qt::green)));
	
	scene.update();
	
	//select the best subtarget and return coordinates
}



void SpecificWorker::goal(){
	if(coor.sub==true){
		qDebug()<<"Voy al subobjetivo";
		mem=inner->transform("base",QVec::vec3(coor.subtarget.x,coor.subtarget.y,coor.subtarget.z),"world");
	}
	else{
		qDebug()<<"Voy al objetivo";
		mem=inner->transform("base",QVec::vec3(coor.target.x,coor.target.y,coor.target.z),"world");
	}
	modulo=mem.norm2();
	if(modulo<400){
		stop();
		if(coor.sub==true){
			qDebug()<<"Alcanzado subobjetivo";
			state = State::FREEWAY;
			coor.sub=false;
		}
		else{
			qDebug()<<"TERMINE";
			state = State::FINISH;
			
		}
	}
	else{
			state = State::FREEWAY;/*
		if(coor.sub==true){
			state = State::ADVANCE;
		}
		else{
			state = State::FREEWAY;
		}*/
		
	}
}

void SpecificWorker::freeway(){
	float rot;
	contador=0;
	punto=0;
	encontrado=false;
	mem=inner->transform("base",QVec::vec3(coor.target.x,coor.target.y,coor.target.z),"world");//Q
	rot=atan2(mem.x(),mem.z());
	bool b= calcularCamino(mem);
	while(!encontrado){
		if(rot>ldata[contador].angle){
			encontrado=true;
			punto=contador;
			qDebug()<<"punto "<< punto;
		}
		contador++;
	}
	if(modulo <= ldata[punto].dist)
	{			
		qDebug()<<"avanza";
		state = State::ADVANCE;
		coor.sub=false;
	}else
	{
		qDebug()<<"subobjetivo";
		state = State::NEWWAY;
	}
}
 bool SpecificWorker::calcularCamino(QVec v){
	 float x, z;
	 float nx, nz, tx, tz;
	 float modulo, modulon, modulot;
	 bool terminar, terminarIzquierda; 
	for(float i=0.1; i<=1; i+0.1){
		x= v.x()*i;
		z= v.z()*i;
		modulo = sqrt(pow(x,2)+pow(z,2));
		nx=(z/modulo)*200;
		nz=(x/modulo)*200;
		terminar=false;
		int j=49;
		modulon = sqrt(pow(nx,2)+pow(nz,2));
		while(!terminar){
			
			tx=(ldata[j].dist*sin(ldata[j].angle))*i;
			tz=(ldata[j].dist*cos(ldata[j].angle))*i;
			modulot = sqrt(pow(tx,2)+pow(tz,2));
			if(nx==tx && nz==tz){
				terminar=true;
				if(modulot<modulon){				
					return false;
				}
			}
			j++;
		}		
		nx=((z/modulo)*200)*(-1);
		nz=(x/modulo)*200;
		terminarIzquierda=false;
		j=49;
		modulon = sqrt(pow(nx,2)+pow(nz,2));
		while(!terminarIzquierda){
			
			tx=(ldata[j].dist*sin(ldata[j].angle))*i;
			tz=(ldata[j].dist*cos(ldata[j].angle))*i;
			modulot = sqrt(pow(tx,2)+pow(tz,2));
			if(nx==tx && nz==tz){
				terminarIzquierda=true;
				if(modulot<modulon){				
					return false;
				}
			}
			j--;
		}		
	}	
	return true;
}
void SpecificWorker::newway(){
	encontradode = false;
	encontradoiz = false;
	contador = 48;
	punto=0;
	const float R = 550; //Robot radius
	float x,z;
	int i,j;
	if(!coor.sub){
	for(i=ldata.size()/2; i>0; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			encontradode = true;
			break;
		}
	}
	for(j=ldata.size()/2; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			encontradode = true;
			break;
		}
	}
	/*while(!encontrado && contador != 3)
	{
		if(fabs(ldata[contador].dist - ldata[contador+1].dist) > 200)
		{
			encontrado = true;
			punto = contador - 3;
		}
		contador--;				
	}
	contador = 50;
	while(!encontrado && contador != 96)
	{
		if(fabs(ldata[contador].dist - ldata[contador-1].dist) > 200)
		{
			encontrado = true;
			punto = contador + 3;
		}
		contador++;				
	}*/
	if(encontradode=true){
		punto=i;
	}
	else{
		if(encontradoiz=true){
			punto=j;
		}
	}
	x=((ldata[punto].dist/3)*2)*sin(ldata[punto].angle);
	z=((ldata[punto].dist/3)*2)*cos(ldata[punto].angle);
	mem=inner->transform("world",QVec::vec3(x,0,z),"base");
	coor.subtarget.x=mem.x();
	coor.subtarget.y=mem.y();
	coor.subtarget.z=mem.z();
	std::cout << "target: " << coor.subtarget.x << ", "  << coor.subtarget.y<< ", " << coor.subtarget.z << ".\n " << std::endl;
	coor.sub = true;
	}
	state = State::ADVANCE;
}


void SpecificWorker::advance(){
	float k= 0.7;
	if(coor.sub){
		mem=inner->transform("base",QVec::vec3(coor.subtarget.x,coor.subtarget.y,coor.subtarget.z),"world");
	}
	else{
		mem=inner->transform("base",QVec::vec3(coor.target.x,coor.target.y,coor.target.z),"world");
	}
	angulo=atan2(mem.x(),mem.z());
	//distancia=fmodf((mem.norm2()*k),float(300));
	differentialrobot_proxy->setSpeedBase(200, angulo); 
	usleep(500000);
	state = State::GOAL;
}

float SpecificWorker::go(const TargetPose &target)
{
	qDebug()<<"Nuevas coordenadas";
	coor.target.x = target.x;
	coor.target.y = target.y;
	coor.target.z = target.z;
	coor.sub=false;
	coor.newtarget=true;
	std::cout << "target: " << coor.target.x << ", "  << coor.target.y<< ", " << coor.target.z << ".\n " << std::endl;
	state = State::INIT;
}

NavState SpecificWorker::getState()
{
	return estado;
}

void SpecificWorker::stop()
{
	
	differentialrobot_proxy->setSpeedBase(0, 0);
}






