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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }


    innerModel = new InnerModel("/home/celia/robocomp/files/innermodel/informatica.xml");
	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);

	RobotInitX = robotState.x;
	RobotInitZ = robotState.z;
	timer.start(Period);



	return true;
}

void SpecificWorker::compute()
{


	TBaseState robotState;
	differentialrobot_proxy->getBaseState(robotState);
	innerModel->updateTransformValues("base", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);
	obtenerMarcas();

	//computeCODE
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


switch(state){

	case State::INIT: {

			qDebug()<< "GO TO POINT";

				const string nodo;
				float angle;

 				state = State::WAIT;
				qDebug()<< "GOING TO" << T.tagx << " , " << T.tagz;
				gotopoint_proxy->go(nodo, T.tagx, T.tagz, angle);
            }
	break;

	case State::WAIT:

            qDebug()<< "WAIT";

            if(gotopoint_proxy->atTarget()){

                current = (current+1)%4 ;
                T.vaciarTag();
                gotopoint_proxy->stop();

            }

            	gotopoint_proxy->turn(0.5);


            break;

    case State::SEARCH:

            qDebug()<< "SEARCH";

            if (!T.esVacio()){
            	state = State::INIT;
            	gotopoint_proxy->stop();
            }

            state= State::FINISH;


            break;

    case State::FINISH:
        qDebug()<<"FINISH";
        qDebug()<< "Posicion del tag" << T.tagx << " , " << T.tagz;
        state = State::SEARCH;
        break;


}
}

int SpecificWorker::obtenerTag(int tg){

	for(unsigned int i = 0; i < listaMarcas.size(); i++)
		if (listaMarcas[i].id == tg)
			return i;

	return -1;

}

 void SpecificWorker::obtenerMarcas(){
 	QVec robot = innerModel->transform("world","base");
	listaMarcas = getapriltags_proxy->checkMarcas();


 	int tag;

    tag = obtenerTag(current);
 		if (tag != -1){
 			QVec targetRobot = innerModel->transform("world",QVec::vec3(listaMarcas[tag].tx, 0, listaMarcas[tag].tz) , "rgbd");
 			T.insertarCoordenadas(listaMarcas[tag].id, targetRobot.x(), targetRobot.z());


 	}




}




