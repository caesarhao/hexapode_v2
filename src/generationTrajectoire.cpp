/**
* @file     generationTrajectoire.cpp
* @brief    le noeud pour recevoir la commande de client et calculer et envoyer les commandes pour le robot.
* @details  comme brief.
* @author   caesarhao@gmail.com
* @copy		Ecole des Mines de Nantes
* @date     2013-03-30
*/
/*
 * FRONT VIEW       ^       0==0             0==0
 *    |=======|     |       |  |---x[___]x---|  |
 *    |       |     Z       |                   |
 *
 *
 *
 * TOP VIEW
 *    0        3
 *    \       /     ^
 *     \_____/      |
 *     |     |      Y
 * 1---|     |---4   X--->
 *     |_____|
 *     /     \      
 *    /       \
 *   2         5
 */

#include <ros/ros.h>
#include <hexapode_v2/translation.h>
#include <hexapode_v2/etat_robot.h>
#include <hexapode_v2/sequence_moteur.h>

#include <iostream>
#include <math.h>
#include "zhrobot.hpp"
using namespace std;
using namespace zhrobot;
// la valeur de servo quand l'angle est 0.
#define SERVO_ZERO	(600)
// la valeur de servo quand l'angle est PI(180 degres).
#define SERVO_PI	(2400)
// le cadre de servo peut bouger.
#define SERVO_CARDE	(SERVO_PI-SERVO_ZERO)
// la valeur de servo quand l'angle est PI/2.
#define SERVO_CENTRE	(1500)
// la maximum degres le servo peut bouger, pas utile maintenant.
#define MAX_ANGLE	(PI/4)
// la maximum distance les pattes peuvent bouger dans l'axe X, Y, Z.
#define MAX_X		(40.0)
#define MAX_Y		(40.0)
#define MAX_Z		(40.0)
// la maximum angle les pattes peuvent tourner horizontalement.
#define MAX_A		(0.30) // environt 17 degrees
// la hauteur des pattes quand ils bouger dans l'air.
#define STEP_UP		(40.0)

// l'etat des pattes de robot.
typedef enum _etat{
	E_ROBOT_PARALLEL,
	E_ROBOT_LEFT_UP, // 1,3,5 up
	E_ROBOT_RIGHT_UP // 0, 2, 4 up
}rbt_etat;

// les types de bouger de robot.
typedef enum _move_state{
	E_MOVE_UPDOWN,
	E_MOVE_VERTICAL,
	E_MOVE_HORIZONTAL,
	E_MOVE_TOURNE
} move_etat;	

void ecouteTranslation(const hexapode_v2::translation trans);

void  envoieCmd();

void ecouteEtatRobot(const hexapode_v2::etat_robot etat_robot);

/**
* @class	GeneTraj
* @brief   la classe pour mettre en modele d'un robot hexapode.
*/
class GeneTraj
{
	private:
		int robotType; ///< le petit(0) ou le grand(1).
		Robot rbs[6]; ///< contenir 6 pattes.
		dVector srcCoor; ///< coordonnee source des pattes, toujours (0, 0, 0, 1).
		dVector dstCoor[6]; ///< les coordonnees des 6 pattes dans le system de coordonnee zero.
		dVector dstCoorStand[6]; ///< les coordonnees initiales des 6 pattes dans le system de coordonnee zero.
		dVector vangles[6]; ///< les angles des servos des pattes.
		rbt_etat retat; ///< l'etat de robot.
		move_etat metat; ///< le type de mouvement de robot.
		char cmd[256]; ///< la commande vers la carte de robot.
		int vMoteur[6][3]; ///< les values numeriques des servos des pattes, calculees de vangles.
		int vMoteurOld[6][3]; ///< les valeurs dernieres de servos.
		double C; ///< la constant valeur dans y = a*x^2 + b*x + C
		double betaX, betaY, betaZ, betaA; ///< la valeur accumulee de commande de client. on obtient la distance de mouvement par y = MAX_Y*sin(betaY);
		
	public:
		GeneTraj();
		~GeneTraj();
		void initRobot1(); ///< initialisation de petit robot.
		void initRobot2(); ///< initialisation de grand robot.
   		char *getCmd(); ///< obtenir la commande pour la carte de robot.
		move_etat calcMoveType(double dx,double dy,double dh, double da); ///< obtenir le type de mouvement de la commande de client.
		void retablirAngles(); ///< reset les angles des pattes.
		int transAngletoNum(int patteNum,int motorNum,double angle); ///< transformer les angles de pattes a les nombres de servos.
		void calcNextPosition(int robotNum); ///< calcuer la position prochaine pour une patte.
		void createTraj(double dBetaX, double dBetaY, double dBetaZ, double dBetaA); ///< creer la trajectoire.
		void createCmd(); ///< creer la commande pour la carte de robot.
		bool checkAngles(dVector angles); ///< pas utile maintenant.
 };


GeneTraj::GeneTraj(){
	robotType = 0; ///< par default, le type de robot est petit.
}
/**
* @brief   fonction pour initialiser le petit robot.
* @param   void
* @return  void
*/
void GeneTraj::initRobot1(){
/**
 *
 *
 *               (2)____(3)
 *                |  a3  |
 * [0]______[1]___|d2    |a4
 *      a1      a2       |
 *                      (4)
*/
	robotType = 0;
	std::cout<<"Initialisation le petit robot"<<std::endl;
	Link ls[]={
		//   theta,    d,		 a, 	     alpha
		Link(0,		0, 		88, 	0), // link1
		Link(0,	  -29,		28,		PI/2), // link2
		Link(0,		0,		77,		0), // link3
		Link(-PI/2,	0,		103,	 0) // link4
	};
	/*
			0	3

		1			4

			2	5
	*/
	for (int i = 0; i < 6; i++){
		rbs[i].setLinks(ls, 4);
		vangles[i].resize(0.0, 4);
	}
	rbs[0].setQ(0, 2.05);
	rbs[0].getLink(0).setA(88);
	rbs[1].setQ(0, PI);
	rbs[1].getLink(0).setA(63);
	rbs[2].setQ(0, -2.05);
	rbs[2].getLink(0).setA(88);
	rbs[3].setQ(0, 1.09);
	rbs[3].getLink(0).setA(88);
	rbs[4].setQ(0, 0);
	rbs[4].getLink(0).setA(63);
	rbs[5].setQ(0, -1.09);
	rbs[5].getLink(0).setA(88);
	
	srcCoor.resize(4, 0.0);
	srcCoor[3] = 1;

	for (int i = 0; i < 6; i++){
		rbs[i].getH();
		dstCoor[i].resize(4, 0.0);
		dstCoorStand[i].resize(4, 0.0);
		dstCoorStand[i] = dstCoor[i] = rbs[i].getDestCoord(srcCoor);
	}
	retat = E_ROBOT_PARALLEL;
	metat = E_MOVE_UPDOWN;
	betaX = betaY = betaZ = betaA = 0;
	C = dstCoorStand[0][2]+STEP_UP;
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}
/**
* @brief   fonction pour initialiser le grand robot.
* @param   void
* @return  void
*/
void GeneTraj::initRobot2(){
/**
 *
 *
 *               (2)___(3)
 *                |  a3 |
 * [0]______[1]___|d2   |a4
 *      a1      a2      |
 *                     (4)
*/
	robotType = 1;
	std::cout<<"Initialisation le grand robot"<<std::endl;

	Link ls[]={
		//   theta, d,		 a, 	alpha
		Link(0,		0, 		136, 	0), // link1
		Link(0,		0,		30,		PI/2), // link2
		Link(0,		0,		57,		0), // link3
		Link(-PI/2,	0,		132,	 0) // link4
	};
	/*
			0	3

		1			4

			2	5
	*/
	for (int i = 0; i < 6; i++){
		rbs[i].setLinks(ls, 4);
		vangles[i].resize(0.0, 4);
	}
	rbs[0].setQ(0, 2*PI/3);
	rbs[1].setQ(0, PI);
	rbs[2].setQ(0, -2*PI/3);
	rbs[3].setQ(0, PI/3);
	rbs[4].setQ(0, 0);
	rbs[5].setQ(0, -PI/3);

	srcCoor.resize(4, 0.0);
	srcCoor[3] = 1;

	for (int i = 0; i < 6; i++){
		rbs[i].getH();
		dstCoor[i].resize(4, 0.0);
		dstCoorStand[i].resize(4, 0.0);
		dstCoorStand[i] = dstCoor[i] = rbs[i].getDestCoord(srcCoor);
	}
	retat = E_ROBOT_PARALLEL;
	metat = E_MOVE_UPDOWN;
	betaX = betaY = betaZ = betaA = 0;
	C = dstCoorStand[0][2]+STEP_UP;
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}

GeneTraj::~GeneTraj(){

}
char* GeneTraj::getCmd(){
	return cmd;
}


int GeneTraj::transAngletoNum(int patteNum, int motorNum, double angle){
	if (patteNum < 3)// gauche
	{
		switch(motorNum){
			case 0:
				return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
			case 1:
				if (0 == robotType){ // petit robot.
					return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{ // grand robot.
					return (int)(-angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
			default: // 2.
				if (0 == robotType){
					return (int)(-(PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{
					return (int)((PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
		}
	}
	else{ // droite.
		switch(motorNum){
			case 0:
				return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
			case 1:
				if (0 == robotType){
					return (int)(-angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{
					return (int)(angle*SERVO_CARDE/PI + SERVO_CENTRE);
				}
			default: // 2
				if (0 == robotType){
					return (int)((PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
				else{
					return (int)(-(PI/2 + angle)*SERVO_CARDE/PI + SERVO_CENTRE);
				}
		}	
	}
}
/**
* @brief   fonction pour creer la commande pour la carte de robot, en fonction de la datasheet.
* @param   void
* @return  void
*/
void GeneTraj::createCmd(){
	cmd[0] = '\0';
	char temp[20];
	for (int i = 0; i<6; i++){
		for (int j = 0; j < 3; j++){
			if (i<3){
				sprintf(temp, "# %d P%d ", 4*i+j, vMoteur[i][j]);
			}
			else{
				sprintf(temp, "# %d P%d ", 4*(1+i)+j, vMoteur[i][j]);			
			}
			if (abs(vMoteur[i][j] - vMoteurOld[i][j]) > 5){
				strcat(cmd, temp);
			}
			vMoteurOld[i][j] = vMoteur[i][j];
		}
	}
	strcat(cmd, "\r\n");
}
// pas utile maintenant.
bool GeneTraj::checkAngles(dVector angles){
	if ((angles[1] >= MAX_ANGLE) || (angles[1]<= -MAX_ANGLE)){
		return false;
	}
	else if((angles[2] >= MAX_ANGLE) || (angles[2]<= -MAX_ANGLE)){
		return false;
	}
	else if(((angles[3]+PI/2) >= MAX_ANGLE) || ((angles[3]+PI/2) <= -MAX_ANGLE)){
		return false;
	}
	else{
		return true;
	}
}
/**
* @brief   fonction pour calculer le type de mouvement de robot, en fonction de la commande de client.
* @param   dx, dy, dh, da
* @return  void
*/
move_etat GeneTraj::calcMoveType(double dx, double dy, double dh, double da){
	if (abs(dx)> 0.001){
		return E_MOVE_HORIZONTAL;
	}
	else if (abs(dy)> 0.001){
		return E_MOVE_VERTICAL;
	}
	else if (abs(dh) > 0.001){
		return E_MOVE_UPDOWN;
	}
	else{
		return E_MOVE_TOURNE;
	}
}
/**
* @brief   fonction pour mettre a zero les angles des pattes.
* @param   void
* @return  void
*/
void GeneTraj::retablirAngles(){
	int i = 0;
	for (i = 0; i < 6; i++){
		rbs[i].setQ(1, 0);
		rbs[i].setQ(2, 0);
		rbs[i].setQ(3, -PI/2);
		dstCoor[i] = rbs[i].getDestCoord(srcCoor);
	}
	betaX = betaY = betaZ = betaA = 0;
	memset(vMoteurOld, 0, sizeof(vMoteurOld));
}
/**
* @brief   fonction pour calculer la position prochaine.
* @param   int
* @return  void
*/
void GeneTraj::calcNextPosition(int robotNum){
	double a, x, y, factor;
	switch(metat){
		case E_MOVE_UPDOWN:
			// justement changer la hauteur des pattes.
			dstCoor[robotNum][2] = dstCoorStand[robotNum][2] - MAX_Z*sin(betaZ);
			break;
			
		case E_MOVE_VERTICAL:
			// factor est 'a' dans y = a*x^2 + b * x + c.
			factor = -(STEP_UP/(MAX_Y*MAX_Y));
			// obtenir y de betaY.
			y = MAX_Y*sin(betaY);
			if (cos(betaY) > 0){// 1 3 5 dans space, 0 2 4 sur le sol.
				if ((robotNum%2) != 0){ // 1 3 5, dans space.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] + y;
					dstCoor[robotNum][2] = factor*(y*y)+C;
				}
				else{ // sur le sol.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] - y;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5, sur le sol.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] + y;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{ // dans space.
					dstCoor[robotNum][1] = dstCoorStand[robotNum][1] - y;
					dstCoor[robotNum][2] = factor*(y*y)+C;
				}
			}
			break;
		case E_MOVE_HORIZONTAL: // c'est la meme que verticale.
			factor = -(STEP_UP/(MAX_X*MAX_X));
			x = MAX_X*sin(betaX);
			if (cos(betaX) > 0){
				if ((robotNum%2) != 0){ // 1 3 5
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] + x;
					dstCoor[robotNum][2] = factor*(x*x)+C;
				}
				else{
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] - x;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] + x;
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{
					dstCoor[robotNum][0] = dstCoorStand[robotNum][0] - x;
					dstCoor[robotNum][2] = factor*(x*x)+C;
				}
			}
			break;
		case E_MOVE_TOURNE:
			factor = -(STEP_UP/(MAX_A*MAX_A));
			a = MAX_A*sin(betaA);
			// obtenir la longueur de patte. len = sqrt(x^2+y^2).
			double len = sqrt(dstCoorStand[robotNum][0]*dstCoorStand[robotNum][0] + 
				dstCoorStand[robotNum][1]*dstCoorStand[robotNum][1]);
			// obtenir l'angle normale de patte.
			double angleNorm = rbs[robotNum].getLink(0).getParaTheta();
			double angle;
			if (cos(betaA) > 0){// 1 3 5 dans space, 0 2 4 sur le sol.
				if ((robotNum%2) != 0){ // 1 3 5, dans space.
					angle = angleNorm + a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = factor*(a*a)+C;
					//cout << "y: " << y << " a: " << a << " z: " << dstCoor[robotNum][2] <<endl;
				}
				else{ // sur le sol.
					angle = angleNorm - a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
			}
			else{
				if ((robotNum%2) != 0){ // 1 3 5, sur le sol.
					angle = angleNorm + a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = dstCoorStand[robotNum][2];
				}
				else{ // dans space.
					angle = angleNorm - a;
					dstCoor[robotNum][0] = len*cos(angle);
					dstCoor[robotNum][1] = len*sin(angle);
					dstCoor[robotNum][2] = factor*(a*a)+C;
				}
			}
			break;
	}
}
/**
* @brief   fonction pour creer la trajectoire.
* @param   double, double, double, double
* @return  void
*/
void GeneTraj::createTraj(double dBetaX, double dBetaY, double dBetaZ, double dBetaA){
	int i = 0, j = 0;
	move_etat met = calcMoveType(dBetaX,dBetaY,dBetaZ, dBetaA);
	if (met != metat){// si le type de mouvement est different, le changer.
		retablirAngles();
		metat = met;
		retat = E_ROBOT_PARALLEL;
	}
	switch (retat){
		case E_ROBOT_PARALLEL: // 
			switch(metat){
				case E_MOVE_UPDOWN:
					betaZ += dBetaZ;
					break;
				case E_MOVE_HORIZONTAL:
				case E_MOVE_VERTICAL:
					betaX += dBetaX;
					betaY += dBetaY;
					retat = E_ROBOT_LEFT_UP;
					break;
				case E_MOVE_TOURNE:
					betaA += dBetaA;
					retat = E_ROBOT_LEFT_UP;
					break;
			}
			break;
		case E_ROBOT_LEFT_UP: // 0,,2,4 work
		case E_ROBOT_RIGHT_UP: // 1,3,5 work
			betaX += dBetaX;
			betaY += dBetaY;
			betaA += dBetaA;
			break;
	}
	for (i = 0; i<6;i++){
		calcNextPosition(i); // calculer la position prochaine pour chaque patte.
		printValarray(dstCoor[i]);
		vangles[i] = rbs[i].invertCoord(dstCoor[i]);
		printValarray(vangles[i]);
		for (j = 0; j < 3; j++){
			vMoteur[i][j] = transAngletoNum(i, j, vangles[i][1+j]);	
		}
	}
	cout << "---------------" << endl;
}

GeneTraj gen;
ros::ServiceClient *pclient = NULL;

void ecouteTranslation(const hexapode_v2::translation trans)
{
	double factor = 15;
    if(1)//!envoie)
    {
        ROS_DEBUG("dx: [%f]", trans.dx);
        ROS_DEBUG("dy: [%f]", trans.dy);
        ROS_DEBUG("da: [%f]", trans.da);
        ROS_DEBUG("dh: [%f]", trans.dh);
		cout << trans.dx << trans.dy << trans.dh << trans.da << endl;
        gen.createTraj(trans.dx/factor, trans.dy/factor, trans.dh/factor, trans.da/factor);
		
     	gen.createCmd();
        envoieCmd();
    }
}


void ecouteEtatRobot(const hexapode_v2::etat_robot etat_robot)
{
    //etat=etat_robot.is_ok;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generationtrajectoire");
	if (argc > 1){
		if (0 == strcmp(argv[1], "0")){
			gen.initRobot1();
		}
		else{
			gen.initRobot2();
		}
	}
	else{
		gen.initRobot1();
	}
	ros::NodeHandle n;
	
	ros::ServiceClient client = n.serviceClient<hexapode_v2::sequence_moteur>("sequence_");
    ros::Subscriber trans_sub = n.subscribe("translation", 1000, ecouteTranslation);
    ros::Subscriber etat_sub = n.subscribe("etat_robot", 1000, ecouteEtatRobot);
	pclient = &client;
    ros::spin();
    return 0;

}


void  envoieCmd()
{

	ROS_DEBUG("ENVOIE SEQUENCE");
	hexapode_v2::sequence_moteur seq_srv;
	seq_srv.request.sequence = gen.getCmd();
	//cout << gen.getCmd();
	seq_srv.request.dim = seq_srv.request.sequence.size()+2;
	pclient->call(seq_srv);
}





