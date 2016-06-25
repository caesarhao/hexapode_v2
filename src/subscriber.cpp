/**
* @file     subscriber.cpp
* @brief    le noeud pour recevoir les commandes et les envoyer a la carte de robot.
* @details  comme brief.
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2013-03-20
*/
#include "ros/ros.h"
#include "hexapode_v2/sequence_moteur.h"
#include <cereal_port/CerealPort.h>
#include <math.h>
#include <string.h>

#define REPLY_SIZE 8
#define TIMEOUT 1000


cereal::CerealPort device; ///< la device de la porte serie.
char sequ[256]={0}; ///< array pour envoyer la commande.
char reply[256];
/**
* @brief   fonction pour verifier si la porte serie est libre ou pas, ca depend de la carte de robot..
* @param   void
* @return  bool
*/
bool isSerialFree(){
	strcpy(sequ, "Q\r\n");
	device.write(sequ, 3);
	memset(reply, 0, 4);
	try{ 
		device.read(reply, 1, 10); 
	}
    catch(cereal::TimeoutException& e){
        ROS_ERROR("Timeout!");
    }
    ROS_INFO("%s", reply);
	if ('+' == reply[0]){ // busy.
		return false;
	}
	else{//'.', free.
		return true;
	}
}
/**
* @brief   callback fonction pour envoyer la commande a la carte de robot.
* @param   hexapode_v2::sequence_moteur::Request&
* @param   hexapode_v2::sequence_moteur::Response&
* @return  bool
*/
bool sequence_moteur(hexapode_v2::sequence_moteur::Request  &req, hexapode_v2::sequence_moteur::Response &res)
{
    int i = 0;
	for(i = 0; i < 100; i++){
		if (isSerialFree()){
			break;
		}
		else{
			usleep(5000); // 5 millisecondes
		}
	}
    strcpy(sequ, (req.sequence).c_str());

    device.write(sequ, (int)(req.dim));
    ROS_INFO("Ecriture sur le port serie "+req.dim);
	ROS_INFO(sequ);
    // Get the reply, the last value is the timeout in ms
	/*
    try{ device.read(reply, 256, TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    ROS_INFO("Reponse obtenu du port serie: %s", reply);
	*/
    ros::spinOnce();

    return true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("sequence_", sequence_moteur);

	
	ROS_INFO("Ouverture du port serie");
    try{ device.open("/dev/ttyUSB0", 115200); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Echec de l'ouverture du port serie!!!");
        ROS_BREAK();
    }
    ROS_INFO("Le port serie est ouvert.");
	
    ROS_INFO("Ready to send data");
    ros::spin();

    return 0;
}
