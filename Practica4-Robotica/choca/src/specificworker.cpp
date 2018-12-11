/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
state=State::INIT;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
}
	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{


	  RoboCompGenericBase::TBaseState robotState;
      RoboCompLaser::TLaserData lData;


      try
      {
        differentialrobot_proxy->getBaseState(robotState);
        lData =laser_proxy->getLaserData();
        innerModel->updateTransformValues("base", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
        QVec inicio;

       switch(state)
       {
        case State::INIT:
    	qDebug()<< "INIT";

    	if(T.isActivo()){
    	  inicio = QVec::vec3(robotState.x, 0, robotState.z);
    	  linea = QLine2D(inicio, T.extraerCoordenadas());
    	  state=State::GOTO;
        }
    	break;

        case State::GOTO:
    	qDebug()<< "GOTO1";
    	goToTarget(lData);
    	break;

        case State::BUG:
        qDebug()<< "BUG";
    	bug(lData, robotState);
        break;

    }
     }catch(const Ice::Exception &ex)
	    {
	        std::cout << ex << std::endl;
	    }




}


void SpecificWorker::goToTarget(const TLaserData& lData)
{
     float vRot;
     float dist;
     QVec rt = innerModel->transform("base", T.extraerCoordenadas(), "world");

     if( obstacle(lData) == true)
     {
         state=State::BUG;
         return;
     }

     if(T.isActivo()==true){
        dist =rt.norm2();

     if( dist > 50)
     {
         vRot = atan2(rt.x(), rt.z());

        if(vRot > 0.5){
             vRot = 0.5;
        }

        if(vRot< -0.5){
             vRot= -0.5;
        }

        differentialrobot_proxy->setSpeedBase(400,vRot);

        }else{
            state = State::INIT;
            differentialrobot_proxy->setSpeedBase(0,0);
            T.setActivo(true);
            return;
        }
     }
}

void SpecificWorker::bug(TLaserData &lData,  TBaseState& robotState)

{
    float rot = 1.0;

     std::sort( lData.begin()+8, lData.end()-8, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; });

       if( lData[8].dist < 350 || lData[lData.size()-8].dist < 350){

       if (lData[8].angle < 0)
         rot = 0.5;
       else
         rot = -0.5;

       differentialrobot_proxy->setSpeedBase(0, 0.5*rot);
     }
     else{
       differentialrobot_proxy->setSpeedBase(300, 0);
       state = State::GOTO;
     }



}



bool SpecificWorker::obstacle(TLaserData lData)

{
     std::sort( lData.begin()+8, lData.end()-8, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; });

    if( lData[8].dist < 350 || lData[lData.size()-8].dist < 350)
      return true;
    else
   return false;
}


bool SpecificWorker::targetAtSight(const TLaserData lData)

{

  QPolygonF polygon;

  for ( auto l: lData){
    QVec lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
    polygon << QPointF(lr.x(), lr.z());
  }

  QVec t = T.extraerCoordenadas();
  return  polygon.containsPoint(QPointF(t.x(), t.z()), Qt::WindingFill);

}


void SpecificWorker::setPick(const Pick &myPick)
{
        qDebug()<<myPick.x<<myPick.z;

    	T.insertarCoordenadas(myPick.x, myPick.z);
    	T.setActivo(true);
        state = State::INIT;

}
