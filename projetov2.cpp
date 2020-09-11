#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h" 
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "FLIE-master/flie.h"

using namespace std;

float feedback, feedbackx, feedbacky, feedbacktheta, ultrasc, ultrase, ultrasd, sensdirext, sensdir, sens, sensesq, sensesqext, lid[512], posicao, distobjeto, slaser, slaserdir, bp1;
float slaserhand1, slaserhand2, slaserhand3, sladodir, sladoesq, difsensor=0, slasermao;

tf::Pose pose;

void subCallback(const nav_msgs::Odometry::ConstPtr& msg) //CRIOU UMA FUNCAO SUBCALLBACK
{

        tf::poseMsgToTF(msg->pose.pose, pose);

	feedbackx = msg->pose.pose.position.x;
	feedbacky = msg->pose.pose.position.y;
	feedbacktheta = tf::getYaw(pose.getRotation());;

}

void Callbacksensorc(const std_msgs::Float32::ConstPtr& msg_ultrassom)
{
  	ultrasc = msg_ultrassom->data;
	//ROS_INFO("ultrasc>>%f\n", ultrasc);
}

void Callbacksensord(const std_msgs::Float32::ConstPtr& msg_ultrassomdir)
{
  	ultrasd = msg_ultrassomdir->data;
	//ROS_INFO("ultrasd>>%f\n", ultrasd);
}

void Callbacksensore(const std_msgs::Float32::ConstPtr& msg_ultrassomesq)
{
  	ultrase = msg_ultrassomesq->data;
	//ROS_INFO("ultrase>>%f\n", ultrase);
}

void Callbacklaserdir(const std_msgs::Float32::ConstPtr& msg_laserdir)
{
  	slaserdir = msg_laserdir->data;
	//ROS_INFO("laserdir>>%f\n", slaserdir);
}

void Callbackladodir(const std_msgs::Float32::ConstPtr& msg_ladodir)
{
  	sladodir = msg_ladodir->data;
	//ROS_INFO("sensor lado dir>>%f\n", sladodir );
}

void Callbackladoesq(const std_msgs::Float32::ConstPtr& msg_ladoesq)
{
  	sladoesq = msg_ladoesq->data;
	//ROS_INFO("sensor lado esq>>%f\n", sladoesq );
}

void Callbacklaserhand1(const std_msgs::Float32::ConstPtr& msg_laserhand1)
{
  	slaserhand1 = msg_laserhand1->data;
	//ROS_INFO("hand1>>%f\n", slaserhand1);
}

void Callbacklaserhand2(const std_msgs::Float32::ConstPtr& msg_laserhand2)
{
  	slaserhand2 = msg_laserhand2->data;
	//ROS_INFO("hand2>>%f\n", slaserhand2);
}
void Callbacklaserhand3(const std_msgs::Float32::ConstPtr& msg_laserhand3)
{
  	slaserhand3 = msg_laserhand3->data;
	//ROS_INFO("hand3>>%f\n", slaserhand3);
}

void Callbacklasermao(const std_msgs::Float32::ConstPtr& msg_lasermao)
{
  	slasermao = msg_lasermao->data;
	//ROS_INFO("sensormao>>%f\n", slasermao);
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
	for (int i = 0; i < 512; i++){
		lid[i] = msg->ranges[i];
		
  }
  float menor = 5;
	  for (int i = 0; i <= 15; i++){  	
			if (lid[i] <= menor&& lid[i]>0){
	  		menor = lid[i];  	
			posicao=i;	
	  		}
		sensdirext = menor;

		//ROS_INFO("dists1ext: %f", menor);
		//ROS_INFO("pontos1ext: %f", posicao);

 	 }
  
  menor = 5;
	  for (int i = 16; i <= 244; i++){  	//248-deu errado - 240 passou muito
			if (lid[i] <= menor&& lid[i]>0){
	  		menor = lid[i];  
			posicao=i;		
	  		}
		sensdir = menor;
		//ROS_INFO("dists1dir: %f", menor);
		//ROS_INFO("pontos1dir: %f", posicao);
	  }
  
  menor = 5;
	  for (int i = 245; i <= 269; i++){  	
			if (lid[i] <= menor&& lid[i]>0){
	  		menor = lid[i]; 
			posicao=i; 		
	  		}
		sens = menor;
 	 }
  
  menor = 5;
	  for (int i = 270; i <= 495; i++){  	
			if (lid[i] <= menor && lid[i]>0){
	  		menor = lid[i];  
			posicao=i;		
		//ROS_INFO("dists1 sensesq: %f", menor);
		//ROS_INFO("pontos1 sensesq: %f", posicao);	
	  		}
		sensesq = menor;
	  }
  
  menor = 5;
	  for (int i = 496; i < 512; i++){  	
			if (lid[i] <= menor && lid[i]>0 ){
	  		menor = lid[i];  	
			posicao=i;	
		//ROS_INFO("dists2 sensesqext: %f", menor);
		//ROS_INFO("pontos2 sensesqext: %f", posicao);
	  		}
		
		sensesqext = menor;

  	 }  

//verificacao da distancia do componente para encontrar o centro

  menor = 5;
	  for (int i = 248; i < 264; i++){  	
			if (lid[i] <= menor && lid[i]>0 ){
	  		menor = lid[i];  	
			posicao=i;	
		//ROS_INFO("dists3: %f", menor);
		//ROS_INFO("pontos3: %f", posicao);
	  		}
		
		distobjeto = menor;

  	 }  

/*ROS_INFO("sensdirext: %f", sensdirext);
ROS_INFO("sensdir: %f", sensdir);
ROS_INFO("sens: %f", sens);
ROS_INFO("sensesq: %f", sensesq);
ROS_INFO("sensesqext: %f", sensesqext);*/
//ROS_INFO("teste: %f", menor);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projetocomlidar");


  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher pubbraco = n.advertise<std_msgs::Float32>("braco", 1000);
  ros::Publisher pubbp1 = n.advertise<std_msgs::Float32>("bp1control", 1000);
  ros::Publisher pubmotor = n.advertise<std_msgs::Float32>("motor", 1000);
  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);
  ros::Subscriber sub1 = n.subscribe("/odom", 1000, subCallback);
  ros::Subscriber subc = n.subscribe("/ultrassom", 1000, Callbacksensorc);
  ros::Subscriber subd = n.subscribe("/ultrassomdir", 1000, Callbacksensord);
  ros::Subscriber sube = n.subscribe("/ultrassomesq", 1000, Callbacksensore);
  ros::Subscriber subh1 = n.subscribe("/laserhand1", 1000, Callbacklaserhand1);
  ros::Subscriber subh2 = n.subscribe("/laserhand2", 1000, Callbacklaserhand2);
  ros::Subscriber subh3 = n.subscribe("/laserhand3", 1000, Callbacklaserhand3);
  ros::Subscriber subld = n.subscribe("/laserbracodir", 1000, Callbacklaserdir);
  ros::Subscriber subdir = n.subscribe("/laserladodireito", 1000, Callbackladodir);
  ros::Subscriber subesq = n.subscribe("/laserladoesq", 1000, Callbackladoesq);
  ros::Subscriber subesmao = n.subscribe("/lasermao", 1000, Callbacklasermao);


  ros::Rate loop_rate(10);

//------------------Sistema Fuzzy para controle da posição-------------------

/*Deve-se definir um sistema de controle que ira conter as regras.*/
fuzzy_control fc1;
fuzzy_control fc2;

/*No programa principal e necessario instanciar as variaveis para conter
todos os conjuntos fuzzy e tambem defini-los.*/

fuzzy_set cat1[15];

/*Deve-se definir as variaveis linguisticas que irao conter os conjuntos fuzzy*/

linguisticvariable Error_pos1, Error_orie1, Control_pos, Control_orie;

/*Deve-se definir as regras de inferencia que irao reger o comportamento do
sistema de controle. Eh necessario instancia-los.*/

rule infrule[25];


/* Define-se os conjuntos fuzzy para a variavel linguistica Error_pos1*/
cat1[0].setname("NG");
cat1[0].setrange(-6,0);
cat1[0].setval(-6, -6, -3, -2);

cat1[1].setname("NP");
cat1[1].setrange(-6,0);
cat1[1].setval(-3, -2, -1, 0);

cat1[2].setname("QZ");
cat1[2].setrange(-6,+6);
cat1[2].setval(-1, 0, 1);

cat1[3].setname("PP");
cat1[3].setrange(0,+6);
cat1[3].setval(0, 1, 2, 3);

cat1[4].setname("PG");
cat1[4].setrange(0,+6);
cat1[4].setval(2, 3, 6, 6);

/*Define-se a Variavel linguistica Error_pos1*/

Error_pos1.setname("Error_pos1");
Error_pos1.includecategory(&cat1[0]);
Error_pos1.includecategory(&cat1[1]);
Error_pos1.includecategory(&cat1[2]);
Error_pos1.includecategory(&cat1[3]);
Error_pos1.includecategory(&cat1[4]);

// Funções de pertinencia Error_orie1
cat1[5].setname("NG");
cat1[5].setrange(-7,0);
cat1[5].setval(-7, -7, -3, -2);

cat1[6].setname("NP");
cat1[6].setrange(-7,0);
cat1[6].setval(-3, -2, -1, 0);

cat1[7].setname("QZ");
cat1[7].setrange(-7,+7);
cat1[7].setval(-1, 0, 1);

cat1[8].setname("PP");
cat1[8].setrange(0,+7);
cat1[8].setval(0, 1, 2, 3);

cat1[9].setname("PG");
cat1[9].setrange(0,+7);
cat1[9].setval(2, 3, 7, 7);

/*Define-se a Variavel linguistica Error_orie1*/

Error_orie1.setname("Error_orie1");
Error_orie1.includecategory(&cat1[5]);
Error_orie1.includecategory(&cat1[6]);
Error_orie1.includecategory(&cat1[7]);
Error_orie1.includecategory(&cat1[8]);
Error_orie1.includecategory(&cat1[9]);

/*Define-se os conjuntos fuzzy para a variavel linguistica Control_pos*/

cat1[10].setname("NB");
cat1[10].setrange(-3,0);
cat1[10].setval(-3, -3, -2, -1);

cat1[11].setname("NS");
cat1[11].setrange(-3,0);
cat1[11].setval(-2, -1, 0);

cat1[12].setname("QZ");
cat1[12].setrange(-3,+3);
cat1[12].setval(-1, 0, 1);

cat1[13].setname("PS");
cat1[13].setrange(0,+3);
cat1[13].setval(0, 1, 2);

cat1[14].setname("PB");
cat1[14].setrange(0,+3);
cat1[14].setval(1, 2, 3, 3);


/*Define-se a variavel linguistica Control*/

Control_pos.setname("Control_pos");
Control_pos.includecategory(&cat1[10]);
Control_pos.includecategory(&cat1[11]);
Control_pos.includecategory(&cat1[12]);
Control_pos.includecategory(&cat1[13]);
Control_pos.includecategory(&cat1[14]);

/*Define-se o metodo defuzzificacao: MAXIMUM, AVERAGEOFMAX, or CENTEROFAREA/CENTROID     */
fc1.set_defuzz(CENTROID);


/* Define-se o fuzzy_control pela entrada fuzzy( Error)
e saida (Control) )*/

fc1.definevars(Error_pos1, Error_orie1, Control_pos);

/*Deve-se incluir cada regra fuzzy no fuzzy_control*/


fc1.insert_rule("NG","NG","QZ"); // If Error_pos IS Negative Grande AND Error_orie IS Negative Grande THEN Control_pos IS Quase Zero
fc1.insert_rule("NG","NP","NS");
fc1.insert_rule("NG","QZ","NB");
fc1.insert_rule("NG","PP","NS");
fc1.insert_rule("NG","PG","QZ");
fc1.insert_rule("NP","NG","QZ"); // If Error_pos IS Negative Pequeno AND Error_orie IS Negative Grande THEN Control_pos IS Quase Zero
fc1.insert_rule("NP","NP","NS");
fc1.insert_rule("NP","QZ","NS");
fc1.insert_rule("NP","PP","NS");
fc1.insert_rule("NP","PG","QZ");
fc1.insert_rule("QZ","NG","QZ"); // If Error_pos IS Quase Zero AND Error_orie IS Negative Grande THEN Control_pos IS Quase Zero
fc1.insert_rule("QZ","NP","QZ");
fc1.insert_rule("QZ","QZ","QZ");
fc1.insert_rule("QZ","PP","QZ");
fc1.insert_rule("QZ","PG","QZ");
fc1.insert_rule("PP","NG","QZ"); // If Error_pos IS Positive Pequeno AND Error_orie IS Negative Grande THEN Control_pos IS Quase Zero
fc1.insert_rule("PP","NP","PS");
fc1.insert_rule("PP","QZ","PS");
fc1.insert_rule("PP","PP","PS");
fc1.insert_rule("PP","PG","QZ");
fc1.insert_rule("PG","NG","QZ"); // If Error_pos IS Positive Grande AND Error_orie IS Negative Grande THEN Control_pos IS Quase Zero
fc1.insert_rule("PG","NP","PS");
fc1.insert_rule("PG","QZ","PB");
fc1.insert_rule("PG","PP","PS");
fc1.insert_rule("PG","PG","QZ");


//----------------Sistema Fuzzy para controle de orientação------------


/*Define-se os conjuntos fuzzy para a variavel linguistica Control_orie*/

cat1[15].setname("NB");
cat1[15].setrange(-6,0);
cat1[15].setval(-6, -6, -4, -2);

cat1[16].setname("NS");
cat1[16].setrange(-6,0);
cat1[16].setval(-4, -2, 0);

cat1[17].setname("QZ");
cat1[17].setrange(-6,+6);
cat1[17].setval(-2, 0, 2);

cat1[18].setname("PS");
cat1[18].setrange(0,+6);
cat1[18].setval(0, 2, 4);

cat1[19].setname("PB");
cat1[19].setrange(0,+6);
cat1[19].setval(2, 4, 6, 6);

/*Define-se a variavel linguistica Control_orie*/

Control_orie.setname("Control_orie");
Control_orie.includecategory(&cat1[15]);
Control_orie.includecategory(&cat1[16]);
Control_orie.includecategory(&cat1[17]);
Control_orie.includecategory(&cat1[18]);
Control_orie.includecategory(&cat1[19]);

fc2.set_defuzz(CENTROID);

fc2.definevars(Error_pos1, Error_orie1, Control_orie);

/*Deve-se incluir cada regra fuzzy no fuzzy_control*/

fc2.insert_rule("NG","NG","NB"); // If Error_pos IS Negative Grande AND Error_orie IS Negative Grande THEN Control_orie IS Negative Big
fc2.insert_rule("NG","NP","NS");
fc2.insert_rule("NG","QZ","QZ");
fc2.insert_rule("NG","PP","PS");
fc2.insert_rule("NG","PG","PB");
fc2.insert_rule("NP","NG","NB"); // If Error_pos IS Negative Pequeno AND Error_orie IS Negative Grande THEN Control_orie IS Negative Big
fc2.insert_rule("NP","NP","NS");
fc2.insert_rule("NP","QZ","QZ");
fc2.insert_rule("NP","PP","PS");
fc2.insert_rule("NP","PG","PB");
fc2.insert_rule("QZ","NG","NB"); // If Error_pos IS Quase Zero AND Error_orie IS Negative Grande THEN Control_orie IS Negative Big
fc2.insert_rule("QZ","NP","NS");
fc2.insert_rule("QZ","QZ","QZ");
fc2.insert_rule("QZ","PP","PS");
fc2.insert_rule("QZ","PG","PB");
fc2.insert_rule("PP","NG","NB"); // If Error_pos IS Positive Pequeno AND Error_orie IS Negative Grande THEN Control_orie IS Negative Big
fc2.insert_rule("PP","NP","NS");
fc2.insert_rule("PP","QZ","QZ");
fc2.insert_rule("PP","PP","PS");
fc2.insert_rule("PP","PG","PB");
fc2.insert_rule("PG","NG","NB"); // If Error_pos IS Positive Grande AND Error_orie IS Negative Grande THEN Control_orie IS Negative Big
fc2.insert_rule("PG","NP","NS");
fc2.insert_rule("PG","QZ","QZ");
fc2.insert_rule("PG","PP","PS");
fc2.insert_rule("PG","PG","PB");


	/*--------------- Sistema Fuzzy para desvio de obstáculos - Velocidade ---------------------------*/	

/*Deve-se definir um sistema de controle que ira conter as regras.*/
fuzzy_control fc_obs;
fuzzy_control fc_obs1;

/*No programa principal e necessario instanciar as variaveis para conter todos os conjuntos fuzzy e tambem defini-los.*/
fuzzy_set cat_obs[12];

/*Deve-se definir as variaveis linguisticas que irao conter os conjuntos fuzzy*/
linguisticvariable Sen_left, Sen_middle, Sen_right, Veloc, Orie;

/*Deve-se definir as regras de inferencia que irao reger o comportamento do sistema de controle. Eh necessario instancia-los.*/
rule infrule_obs[15];

// Funções de pertinência da distância do obstáculo (m).

/* Define-se os conjuntos fuzzy para a variavel linguistica Sen_left*/
cat_obs[0].setname("PT"); //PT - Perto
cat_obs[0].setrange(0,+0.55);
cat_obs[0].setval(0, 0, 0.15, 0.25);

cat_obs[1].setname("MD"); //PT - Médio
cat_obs[1].setrange(0,+0.55);
cat_obs[1].setval(0.15, 0.25, 0.35);

cat_obs[2].setname("LG"); //LG - Longe
cat_obs[2].setrange(0,+0.55);
cat_obs[2].setval(0.25, 0.35, 0.55, 0.55);

/*Define-se a Variavel linguistica Sen_left*/
Sen_left.setname("Sen_left");
Sen_left.includecategory(&cat_obs[0]);
Sen_left.includecategory(&cat_obs[1]);
Sen_left.includecategory(&cat_obs[2]);

/* Define-se os conjuntos fuzzy para a variavel linguistica Sen_middle*/
cat_obs[3].setname("PT"); //PT - Perto
cat_obs[3].setrange(0,+0.55);
cat_obs[3].setval(0, 0, 0.15, 0.25);

cat_obs[4].setname("MD"); //PT - Médio
cat_obs[4].setrange(0,+0.55);
cat_obs[4].setval(0.15, 0.25, 0.35);

cat_obs[5].setname("LG"); //LG - Longe
cat_obs[5].setrange(0,+0.55);
cat_obs[5].setval(0.25, 0.35, 0.55, 0.55);

/*Define-se a Variavel linguistica Sen_middle*/
Sen_middle.setname("Sen_middle");
Sen_middle.includecategory(&cat_obs[3]);
Sen_middle.includecategory(&cat_obs[4]);
Sen_middle.includecategory(&cat_obs[5]);

/* Define-se os conjuntos fuzzy para a variavel linguistica Sen_right*/
cat_obs[6].setname("PT"); //PT - Perto
cat_obs[6].setrange(0,+0.55);
cat_obs[6].setval(0, 0, 0.15, 0.25);

cat_obs[7].setname("MD"); //PT - Médio
cat_obs[7].setrange(0,+0.55);
cat_obs[7].setval(0.15, 0.25, 0.35);

cat_obs[8].setname("LG"); //LG - Longe
cat_obs[8].setrange(0,+0.55);
cat_obs[8].setval(0.25, 0.35, 0.55, 0.55);

/*Define-se a Variavel linguistica Sen_right*/
Sen_right.setname("Sen_right");
Sen_right.includecategory(&cat_obs[6]);
Sen_right.includecategory(&cat_obs[7]);
Sen_right.includecategory(&cat_obs[8]);

/*Define-se os conjuntos fuzzy para a variavel linguistica Veloc*/
cat_obs[9].setname("DV"); // DV - Devagar
cat_obs[9].setrange(0,+0.3);
cat_obs[9].setval(0, 0, 0.10);

cat_obs[10].setname("Md"); // Md - Média
cat_obs[10].setrange(0,+0.3);
cat_obs[10].setval(0, 0.10, 0.20); //cat_obs[10].setval(0, 0.20, 0.40);

cat_obs[11].setname("RP"); // RP - Rápido
cat_obs[11].setrange(0,+0.3);
cat_obs[11].setval(0.10, 0.2, 0.3, 0.3); //cat_obs[11].setval(0.20, 0.04, 0.5, 0.5);

/*Define-se a variavel linguistica Veloc*/
Veloc.setname("Veloc");
Veloc.includecategory(&cat_obs[9]);
Veloc.includecategory(&cat_obs[10]);
Veloc.includecategory(&cat_obs[11]);

/*Define-se o metodo defuzzificacao: MAXIMUM, AVERAGEOFMAX, or CENTEROFAREA/CENTROID*/
fc_obs.set_defuzz(CENTROID);

/* Define-se o fuzzy_control pela entrada fuzzy(Error) e saida (Control))*/
fc_obs.definevars(Sen_left, Sen_middle, Sen_right, Veloc);


/*Deve-se incluir cada regra fuzzy no fuzzy_control*/

fc_obs.insert_rule("PT","PT","PT","DV"); // If Sen_left IS PERTO AND Sens_middle IS PERTO AND Sen_right IS PERTO THEN Veloc IS DEVAGAR.
fc_obs.insert_rule("PT","PT","MD","DV");
fc_obs.insert_rule("PT","PT","LG","DV");
fc_obs.insert_rule("PT","MD","PT","DV");
fc_obs.insert_rule("PT","MD","MD","DV");
fc_obs.insert_rule("PT","MD","LG","DV");
fc_obs.insert_rule("PT","LG","PT","DV");
fc_obs.insert_rule("PT","LG","MD","DV");
fc_obs.insert_rule("PT","LG","LG","DV");
fc_obs.insert_rule("MD","PT","PT","DV");
fc_obs.insert_rule("MD","PT","MD","DV");
fc_obs.insert_rule("MD","PT","LG","DV");
fc_obs.insert_rule("MD","MD","PT","DV");
fc_obs.insert_rule("MD","MD","MD","Md");
fc_obs.insert_rule("MD","MD","LG","Md");
fc_obs.insert_rule("MD","LG","PT","DV");
fc_obs.insert_rule("MD","LG","MD","Md");
fc_obs.insert_rule("MD","LG","LG","Md");
fc_obs.insert_rule("LG","PT","PT","DV");
fc_obs.insert_rule("LG","PT","MD","DV");
fc_obs.insert_rule("LG","PT","LG","DV");
fc_obs.insert_rule("LG","MD","PT","DV");
fc_obs.insert_rule("LG","MD","MD","Md");
fc_obs.insert_rule("LG","MD","LG","Md");
fc_obs.insert_rule("LG","LG","PT","DV");
fc_obs.insert_rule("LG","LG","MD","RP");
fc_obs.insert_rule("LG","LG","LG","RP");


/*Define-se os conjuntos fuzzy para a variavel linguistica Orie*/
cat_obs[12].setname("NB");
cat_obs[12].setrange(-2.5,0);
cat_obs[12].setval(-2.5, -2.5, -1.5, -1);

cat_obs[13].setname("NS");
cat_obs[13].setrange(-2.5,0);
cat_obs[13].setval(-1.5, -1, -0);

cat_obs[14].setname("QZ");
cat_obs[14].setrange(-2.5,+2.5);
cat_obs[14].setval(-1, 0, 1);

cat_obs[15].setname("PS");
cat_obs[15].setrange(0,+2.5);
cat_obs[15].setval(0, 1, 1.5);

cat_obs[16].setname("PB");
cat_obs[16].setrange(0,+2.5);
cat_obs[16].setval(1, 1.5, 2.5, 2.5);

/*Define-se a variavel linguistica Orie*/
Orie.setname("Orie");
Orie.includecategory(&cat_obs[12]);
Orie.includecategory(&cat_obs[13]);
Orie.includecategory(&cat_obs[14]);
Orie.includecategory(&cat_obs[15]);
Orie.includecategory(&cat_obs[16]);

/*Define-se o metodo defuzzificacao: MAXIMUM, AVERAGEOFMAX, or CENTEROFAREA/CENTROID*/
fc_obs1.set_defuzz(CENTROID);

/* Define-se o fuzzy_control pela entrada fuzzy(Error) e saida (Control))*/
fc_obs1.definevars(Sen_left, Sen_middle, Sen_right, Orie);

/*Deve-se incluir cada regra fuzzy no fuzzy_control*/
fc_obs1.insert_rule("PT","PT","PT","NB"); // If Sen_left IS PERTO AND Sens_middle IS PERTO AND Sen_right IS PERTO THEN Orie IS POSITIVE BIG.
fc_obs1.insert_rule("PT","PT","MD","NB");
fc_obs1.insert_rule("PT","PT","LG","NB");
fc_obs1.insert_rule("PT","MD","PT","NB");
fc_obs1.insert_rule("PT","MD","MD","NB");
fc_obs1.insert_rule("PT","MD","LG","NB");
fc_obs1.insert_rule("PT","LG","PT","NB");
fc_obs1.insert_rule("PT","LG","MD","NB");
fc_obs1.insert_rule("PT","LG","LG","NB");
fc_obs1.insert_rule("MD","PT","PT","PB");
fc_obs1.insert_rule("MD","PT","MD","NB");
fc_obs1.insert_rule("MD","PT","LG","NB");
fc_obs1.insert_rule("MD","MD","PT","PB");
fc_obs1.insert_rule("MD","MD","MD","NB");
fc_obs1.insert_rule("MD","MD","LG","NS");
fc_obs1.insert_rule("MD","LG","PT","PB");
fc_obs1.insert_rule("MD","LG","MD","NS");
fc_obs1.insert_rule("MD","LG","LG","NS");
fc_obs1.insert_rule("LG","PT","PT","PB");
fc_obs1.insert_rule("LG","PT","MD","PB");
fc_obs1.insert_rule("LG","PT","LG","NB");
fc_obs1.insert_rule("LG","MD","PT","PB");
fc_obs1.insert_rule("LG","MD","MD","PS");
fc_obs1.insert_rule("LG","MD","LG","NS");
fc_obs1.insert_rule("LG","LG","PT","PB");
fc_obs1.insert_rule("LG","LG","MD","PS");
fc_obs1.insert_rule("LG","LG","LG","NB");



  if (ros::ok())
  {


    geometry_msgs::Twist msg;
    std_msgs::Float32 msgbraco;
    std_msgs::Float32 msgbp1;
    std_msgs::Float32 msgmotor;
    float posdesejada[1], oridesejada, Erro_dist=99, Erro_orie=99,destino, destax, destay;
    float tolerance_orie = 0.005, tolerance_pos = 0.05; //TRABALHA COM AS DUAS COORDENADAS
    float angulo, braco=0, pararbraco=0, ligarbraco=0, ultraalterado, controlbp1=250, laseralterado=0, motor=0, fechargarra=0, controlgarra=0, alturadoobjeto1, alturaesfera=0, objeto1=0;
    int ang1, ang2;
   	
    msgbraco.data=braco;
    msgmotor.data=motor;

    //system("rosservice call reset");

    cout << "Voce deseja pegar um objeto? Digite 1 para SIM e 2 para nao?\nX>>";
    cin >> posdesejada[0]; //ARMAZENA A COORDENADA X DESEJADA

	destino = posdesejada[0];
	if (destino==2){
		return 0;
	}

	if (destino==1){
		destax=4;
		destay=1.35;
		objeto1=1;
	}
    
 	
     if (sensesqext < 0.05 ||  sensesq < 0.05 ){
       ultraalterado = 1;
	ros::spinOnce();
        }

     if (sensesqext < 0.05 ||  sensesq < 0.05 ){
       ultraalterado = 1;
	ros::spinOnce();
        }


    ros::spinOnce();

	
    // Controle da orientacao e posicao
 	while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos) && ultraalterado>0.75 ) { //& ultraalterado

		 angulo = atan2(destay-feedbacky,destax-feedbackx);

		Erro_dist =  sqrt(pow(destax-feedbackx,2)+pow(destay-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 		

	if ((sensesqext < 0.85 && sensesqext > 0.1)  ||  (sensesq < 0.6 && sensesq > 0.1 )){
        ultraalterado = 0.74;
	
        }
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        

//	ROS_INFO("Sensor1>>%f\n", Sensores[0]);
//	ROS_INFO("Sensor2>>%f\n", Sensores[1]);
//	ROS_INFO("Sensor3>>%f\n", Sensores[2]);
    			
			        

				if ((Ultrassom[0] < 0.3)||(Ultrassom[1] < 0.3)||(Ultrassom[2] < 0.3)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		braco=0;

         	//ROS_INFO("theta>>%f,Erro>>%f",feedbacktheta,erroorie);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
			
		        msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;
			

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

			while ( ((sensesqext < 0.75 && sensesqext > 0.1)  ||  (sensesq < 0.75 && sensesq > 0.1 )) && objeto1==1){ //sensesq < 0.65 && sensesq > 0.1
			braco=0;
			motor=5;
		        msg.linear.x = 0.05;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("posicionando robo sensoresq>>%f\n",sensesq);
			ROS_INFO("posicionando robo sensoresqext>>%f\n",sensesqext);
			ROS_INFO("posicionando robo sens>>%f\n",sens);

			}

			while ( (distobjeto < 0.8 && distobjeto > 0.1) ){
			braco=1;
			motor=6;
		        msg.linear.x = 0.025;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			difsensor = abs(sladodir-sladoesq);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("diferenca entre lado dir e esq>>%f\n", difsensor);
			ROS_INFO("altura braco braco>>%f\n",slasermao);
			}

	
			while ( ((slaserdir < 0.4 && laseralterado < 1) || difsensor>0.0005) ){
			braco=2;
			motor=6;

			if(difsensor>0.0035 || slaserdir < 0.08){
				if(sladodir>sladodir){
		        		msg.linear.x = -0.05;
		       			msg.angular.z = 5;	
				}				
				if(sladodir<sladodir){
		        		msg.linear.x = -0.05;
		       			msg.angular.z = -5;	
				}
				if(slaserdir < 0.08){
		        		msg.linear.x = -0.01;
		       			msg.angular.z = 0;	
				}
			}
			else{
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			}
			msgmotor.data=motor;
			msgbraco.data=braco;
	       		pub.publish(msg);
			pubmotor.publish(msgmotor);
			pubbraco.publish(msgbraco);
			
			difsensor = abs(sladodir-sladoesq);
			ros::spinOnce();
		        loop_rate.sleep();
			//ROS_INFO("posicionando garra e robo para frente>>%f\n",slaserdir);
			//ROS_INFO("laser hand1>>%f\n",slaserhand1);
			//ROS_INFO("laser hand2>>%f\n",slaserhand2);
			//ROS_INFO("laser hand3>>%f\n",slaserhand3);
			pararbraco=0;
			       ROS_INFO("valor do sensor laser>>%f\n",slaserdir);
			ROS_INFO("diferenca entre lado dir e esq>>%f\n", difsensor);
			        ROS_INFO("altura do braco>>%f\n", slasermao);

			if (slaserdir < 0.035){
       			laseralterado = 1;
			ros::spinOnce();}

			}
			
			while(((slaserhand2<0.025 && slaserhand2>0.0108) || (slaserhand3<0.025 && slaserhand3>0.0708)) || fechargarra<100){ 
				
				controlgarra = abs(slaserhand2 - slaserhand3);

				braco=3;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				fechargarra=fechargarra+1;
				ligarbraco=0;
				alturadoobjeto1=slasermao;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("controle da garra>>%f\n", controlgarra);
			        ROS_INFO("sensor hand1>>%f\n", slaserhand1);
			        ROS_INFO("sensor hand2>>%f\n", slaserhand2);
			        ROS_INFO("sensor hand3>>%f\n", slaserhand3);
			        ROS_INFO("valor do braco>>%f\n", braco);
			        //ROS_INFO("altura do objeto>>%f\n", alturadoobjeto1);
				alturadoobjeto1=slasermao;

			}


			while ( (distobjeto < 0.8 || ligarbraco<400 ) ){
			braco=4;
			motor=6;
			ligarbraco = ligarbraco+1;
		        msg.linear.x = 0.05;
		        msg.angular.z = 0.1;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("levantar braco>>%f\n", ligarbraco);

			}
			
			while(pararbraco<50){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararbraco=pararbraco+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando braco>>%f\n", pararbraco);
			}


    // Controle da orientacao e posicao para deslocamento do robo ate o ponto B

float pontobx=0.25, pontoby=-1.5, pararrobo=0, baixarbraco=0, bracoposinicial=0, abrirgarra=0, alterarpontox0=0;

 	while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos) ) { //& ultraalterado

		 angulo = atan2(pontoby-feedbacky,pontobx-feedbackx);

		Erro_dist =  sqrt(pow(pontobx-feedbackx,2)+pow(pontoby-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 		


	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

	if (((sens < 1.5 && sens > 0.1)  ||  (sensesq < 1.5 && sensesq > 0.1 ) ||  (sensdir < 1.5 && sensdir > 0.1 )) && alterarpontox0==0){        //se tiver um objeto no local
	
		pontobx=pontobx+0.25;
		alterarpontox0=1;

        }

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        


				if ((Ultrassom[0] < 0.3)||(Ultrassom[1] < 0.3)||(Ultrassom[2] < 0.3)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		pararrobo=0;

         	//ROS_INFO("theta>>%f,Erro>>%f",feedbacktheta,erroorie);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
		        msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

			while(pararrobo<10){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo=pararrobo+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo);
			}
			alturadoobjeto1=alturadoobjeto1+0.005;

			while ( (slasermao>=alturadoobjeto1 && pararrobo==10 ) ){
			braco=5;// estava 5
			motor=5;
			baixarbraco = baixarbraco+1;
		        msg.linear.x = 0.02;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			//pararbraco=0;
			//ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("baixar braco>>%f\n", baixarbraco);

			}
// colocado novo
			while(abrirgarra < 100){ 
				braco=6;	//estava 6
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra=abrirgarra+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra);
				ROS_INFO("robo para tras>>%f\n", distobjeto);
			}

			while(distobjeto < 0.9){ 
				braco=6;	//estava 6
				motor=6;
    		        	msg.linear.x = 0.1;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra=abrirgarra+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra);
				ROS_INFO("robo para tras>>%f\n", distobjeto);
			}

			while(bracoposinicial<50){ 
				braco=7;	
				motor=6;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0.5;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				bracoposinicial=bracoposinicial+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e posicao inicial>>%f\n", bracoposinicial);
			        //ROS_INFO("mensagem para braco>>%f\n", msgbraco);
			}

				while(pararrobo>9 && pararrobo<50){ 
				braco=7;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo=pararrobo+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo);
			}

//Fim do robo pegando e levando um copo do ponto A ate o ponto B

//Inicio para ir buscar um novo copo

float opcao, pontoax, pontoay, pararrobo2=0, baixarbraco2=0, bracoposinicial2=0, abrirgarra2=0, pararbraco2=0, ligarbraco2=0, ultraalterado2=0, laseralterado2=0, fechargarra2=0;

float girarrobo=0, posicaoinicial=0, alturadoobjeto2, objeto2=0;


    cout << "Voce deseja pegar outro objeto? Digite 1 para SIM e 2 para nao?\nX>>";
    cin >> posdesejada[0]; //ARMAZENA A COORDENADA X DESEJADA

	opcao = posdesejada[0];
	if (opcao==2){
		pontoax=0;
		pontoay=0;
	Erro_dist=99;
	Erro_orie=99;
	posicaoinicial=1;
	}

	if (opcao==1){
		pontoax=4;
		pontoay=1.35;
	Erro_dist=99;
	Erro_orie=99;
	posicaoinicial=1;
	objeto2=1;
	}

     if (sensesqext < 0.05 ||  sensesq < 0.05  ||  sensdir < 0.05 ||  sensdirext < 0.05){
       ultraalterado = 1;
	ros::spinOnce();
        }

	
 	
    // Controle da orientacao e posicao

while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos)&& ultraalterado>0.6 ) { //& ultraalterado

		 angulo = atan2(pontoay-feedbacky,pontoax-feedbackx);

		Erro_dist =  sqrt(pow(pontoax-feedbackx,2)+pow(pontoay-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 	

	if ((sensesq < 1 && sensesq > 0.1 ) && posicaoinicial==1){
        ultraalterado = 0.5;
	}	

	if (posicaoinicial==0){
        ultraalterado = 1;
	}
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        

//	ROS_INFO("Sensor1>>%f\n", Sensores[0]);
//	ROS_INFO("Sensor2>>%f\n", Sensores[1]);
//	ROS_INFO("Sensor3>>%f\n", Sensores[2]);
    			
			        

				if ((Ultrassom[0] < 0.4)||(Ultrassom[1] < 0.4)||(Ultrassom[2] < 0.4)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		braco=0;

         	ROS_INFO("valor sensor esq1>>%f,",sensesq);
         	ROS_INFO("valor sensor esqext>>%f,",sensesqext);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
			msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;
			

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

		if(opcao==2){
			  return 0;
		}

			while(pararrobo2<10){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo2=pararrobo2+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando2 robo>>%f\n", pararrobo2);
			}

			while ( ((sensesq < 1 || sens <1 || sensdir<1.1)) && objeto2==1 ){
			braco=0;
			motor=5;
			
  			msg.linear.x = 0.05;
        		msg.angular.z =  0;

			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("distancia do sensor esq>>%f\n", sensesq);
			ROS_INFO("distancia do sensor centro>>%f\n", sens);
			ROS_INFO("distancia do sensor direito>>%f\n", sensdir);
			ROS_INFO("distancia do sensor dir ext>>%f\n", sensdirext);
			
			}

			while ( (sensdirext < 1.07 && sensdirext > 0.7)  ||  (sensdir < 1.07 && sensdir > 0.7 )){  //dirext estava 1.1
			braco=0;
			motor=6;
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			girarrobo=1;
			ROS_INFO("posicionandorobo ate o copo dir>>%f\n",sensdir);
			ROS_INFO("posicionandorobo ate o copo dirext>>%f\n",sensdirext);
			}

			while ( girarrobo<50){
			braco=0;
			motor=5;
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			girarrobo=girarrobo+1;
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("girando o robo>>%f\n",girarrobo);
			}

			while (((sensdirext < 1.1 && sensdirext > 0.7)  ||  (sensdir < 1.1 && sensdir > 0.7 ))&&girarrobo==50){
			braco=0;
			motor=5;
		        msg.linear.x = -0.05;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("centralizando robo com copo dir>>%f\n",sensdir);
			ROS_INFO("centralizando robo com copo dirext>>%f\n",sensdirext);
			}

			while(girarrobo>=50 && girarrobo<70){ 
				braco=1;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				girarrobo=girarrobo+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando2 robo>>%f\n", girarrobo);
			}

			while ( (distobjeto < 1.05 && distobjeto > 0.1) ){
			braco=1;
			motor=6;
		        msg.linear.x = 0.01;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("robo para tras>>%f\n", distobjeto);

			}

			while ( ((slaserdir < 0.7 && laseralterado2 < 1) || difsensor>0.00005) ){
			braco=2;
			motor=6;

			if(difsensor>0.001){
				if(sladodir>sladodir){
		        		msg.linear.x = -0.05;
		       			msg.angular.z = 5;	
				}				
				if(sladodir<sladodir){
		        		msg.linear.x = -0.05;
		       			msg.angular.z = -5;	
				}
			}
			else{
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			}
			msgmotor.data=motor;
			msgbraco.data=braco;
	       		pub.publish(msg);
			pubmotor.publish(msgmotor);
			pubbraco.publish(msgbraco);
			
			difsensor = abs(sladodir-sladoesq);
			ros::spinOnce();
		        loop_rate.sleep();
			//ROS_INFO("posicionando garra e robo para frente>>%f\n",slaserdir);
			//ROS_INFO("laser hand1>>%f\n",slaserhand1);
			//ROS_INFO("laser hand2>>%f\n",slaserhand2);
			//ROS_INFO("laser hand3>>%f\n",slaserhand3);
			pararbraco=0;
			       ROS_INFO("valor do sensor laser>>%f\n",slaserdir);
			ROS_INFO("diferenca entre lado dir e esq>>%f\n", difsensor);

			if (slaserdir < 0.03){
       			laseralterado2 = 1;
			ros::spinOnce();}

			}
			
			while(fechargarra2<150){ 
				
				controlgarra = abs(slaserhand2 - slaserhand3);

				/*if (controlgarra <= 0.001){
				braco=13;
				}
				else{
				braco=3;
				}	*/			
				braco=3;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				fechargarra2=fechargarra2+1;
				alturadoobjeto2=slasermao;
				ligarbraco=0;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("controle da garra>>%f\n", controlgarra);
			        ROS_INFO("valor do braco>>%f\n", braco);
			        ROS_INFO("altura do objeto 2>>%f\n",alturadoobjeto2);

			}
			

			while ( (distobjeto < 0.8 || ligarbraco2<200 ) ){
			braco=4;
			motor=6;
			ligarbraco2 = ligarbraco2+1;
		        msg.linear.x = 0.1;
		        msg.angular.z = 0.5;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco2=0;
			ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("levantar braco>>%f\n", ligarbraco2);

			}
			
			while(pararbraco2<50){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararbraco2=pararbraco2+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando braco>>%f\n", pararbraco2);
			}

// Controle da orientacao e posicao para deslocamento do robo ate o ponto B

float alterarpontox=0, pararrobo3=0, baixarbraco3=0, bracoposinicial3=0, abrirgarra3=0, girandorobo3=0, pararrobo4=0;

 	while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos) ) { //& ultraalterado

		 angulo = atan2(pontoby-feedbacky,pontobx-feedbackx);

		Erro_dist =  sqrt(pow(pontobx-feedbackx,2)+pow(pontoby-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 		

	if (((sens < 1.5 && sens > 0.1)  ||  (sensesq < 1.5 && sensesq > 0.1 ) ||  (sensdir < 1.5 && sensdir > 0.1 )) && alterarpontox==0){        //se tiver um objeto no local
	
		pontobx=pontobx+0.25;
		alterarpontox=1;

        }
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        


				if ((Ultrassom[0] < 0.3)||(Ultrassom[1] < 0.3)||(Ultrassom[2] < 0.3)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		//braco=0;
		pararrobo3=0;

         	//ROS_INFO("theta>>%f,Erro>>%f",feedbacktheta,erroorie);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
		        msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

			while(pararrobo3<10){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo3=pararrobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo);
			}

			alturadoobjeto2=alturadoobjeto2+0.003;

			while ( (slasermao>=alturadoobjeto2 && pararrobo3==10 ) ){
			braco=5;
			motor=5;
			baixarbraco3 = baixarbraco3+1;
		        msg.linear.x = 0.01;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			//pararbraco3=0;
			//ROS_INFO("rotacionar robo>>%f\n", distobjeto);
			ROS_INFO("baixar braco>>%f\n", baixarbraco);

			}

			while(distobjeto < 0.8){ 
				braco=6;	
				motor=6;
    		        	msg.linear.x = 0.1;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				//abrirgarra=abrirgarra+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        //ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra);
				ROS_INFO("robo para tras>>%f\n", distobjeto);
			}

			while(bracoposinicial3<20){ 
				braco=7;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				bracoposinicial3=bracoposinicial3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e posicao inicial>>%f\n", bracoposinicial);
			        //ROS_INFO("mensagem para braco>>%f\n", msgbraco);
			}

				while(girandorobo3<85){ 
				braco=7;	
				motor=6;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0.5;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				girandorobo3=girandorobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("girando robo>>%f\n", girandorobo3);
			}
				while(pararrobo4<20){ 
				braco=7;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo4=pararrobo4+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo4);
			}


//Fim do robo pegando e levando um copo do ponto A ate o ponto B

//Inicio para ir buscar um novo copo

//Inicio para ir buscar um novo copo

float ligarbraco3=0, ultraalterado3=0, laseralterado3=0, fechargarra3=0, controlgarra3=0, pararbraco3=0;
Erro_dist=99;
Erro_orie=99;

baixarbraco3=0;
bracoposinicial3=0;
abrirgarra3=0;
pararrobo3=0;


float girarrobo3=0, posicaoinicial3=0, alturadoobjeto3, objeto3=0;


    cout << "Voce deseja pegar outro objeto? Digite 1 para SIM e 2 para nao?\nX>>";
    cin >> posdesejada[0]; //ARMAZENA A COORDENADA X DESEJADA

	opcao = posdesejada[0];
	if (opcao==2){
		pontoax=0;
		pontoay=0;
	Erro_dist=99;
	Erro_orie=99;
	posicaoinicial=1;
	}

	if (opcao==1){
		pontoax=4;
		pontoay=1.35;
	Erro_dist=99;
	Erro_orie=99;
	posicaoinicial=1;
	ultraalterado3 = 1;
	objeto3=1;
	}

     if (sensesqext < 0.05 ||  sensesq < 0.05  ||  sensdir < 0.05 ||  sensdirext < 0.05){
       ultraalterado3 = 1;
	ros::spinOnce();
        }

	
 	
    // Controle da orientacao e posicao

while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos)&& ultraalterado3>0.6 ) { //& ultraalterado

		 angulo = atan2(pontoay-feedbacky,pontoax-feedbackx);

		Erro_dist =  sqrt(pow(pontoax-feedbackx,2)+pow(pontoay-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 	

	if ((sensesq < 1 && sensesq > 0.1 ) && posicaoinicial==1){
        ultraalterado3 = 0.5;
	}	

	if (posicaoinicial==0){
        ultraalterado3 = 1;
	}
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        

//	ROS_INFO("Sensor1>>%f\n", Sensores[0]);
//	ROS_INFO("Sensor2>>%f\n", Sensores[1]);
//	ROS_INFO("Sensor3>>%f\n", Sensores[2]);
    			
			        

				if ((Ultrassom[0] < 0.4)||(Ultrassom[1] < 0.4)||(Ultrassom[2] < 0.4)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		braco=0;

         	ROS_INFO("valor sensor esq1>>%f,",sensesq);
         	ROS_INFO("valor sensor esqext>>%f,",sensesqext);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
			msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;
			

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

		if(opcao==2){
			  return 0;
		}

			while(pararrobo3<10){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo3=pararrobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando3 robo>>%f\n", pararrobo3);
			}

			while ( ((sensesq < 1.1 || sens <1.1 || sensdir<1.02) && objeto3==1)) {
			braco=0;
			motor=5;
			
  			msg.linear.x = 0.05;
        		msg.angular.z =  0;

			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("distancia do sensor esq>>%f\n", sensesq);
			ROS_INFO("distancia do sensor centro>>%f\n", sens);
			ROS_INFO("distancia do sensor direito>>%f\n", sensdir);
			ROS_INFO("distancia do sensor dir ext>>%f\n", sensdirext);
			
			}

			while ( (sensdirext < 1.1 && sensdirext > 0.7)  ||  (sensdir < 1.1 && sensdir > 0.7 )){  //dirext estava 1.1
			braco=0;
			motor=6;
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			girarrobo3=1;
			ROS_INFO("posicionandorobo ate o copo dir>>%f\n",sensdir);
			ROS_INFO("posicionandorobo ate o copo dirext>>%f\n",sensdirext);
			}

			while ( girarrobo3<50){
			braco=0;
			motor=5;
		        msg.linear.x = -0.1;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			girarrobo3=girarrobo3+1;
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("girando o robo>>%f\n",girarrobo3);
			}

			while (((sensdirext < 1.1 && sensdirext > 0.7)  ||  (sensdir < 1.1 && sensdir > 0.7 ))&&girarrobo3==50){
			braco=0;
			motor=5;
		        msg.linear.x = -0.05;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("centralizando robo com copo dir>>%f\n",sensdir);
			ROS_INFO("centralizando robo com copo dirext>>%f\n",sensdirext);
			}

			while(girarrobo3>=50 && girarrobo3<70){ 
				braco=1;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				girarrobo3=girarrobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando3 robo>>%f\n", girarrobo3);
			}

			while ( (distobjeto < 0.95 && distobjeto > 0.1) ){
			braco=1;
			motor=6;
		        msg.linear.x = 0.01;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			ROS_INFO("robo para tras>>%f\n", distobjeto);

			}

			while((distobjeto < 1.1 && distobjeto >0.95) || alturaesfera == 0){
				
				if (slasermao>0.247 && slaserdir==0){
				braco=16;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);
				}			
				else if ((slasermao<=0.241 && slasermao>0.235)  && slaserdir==0){
				braco=17;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);
				}
	
				else if ((slasermao<=0.235 && slasermao>0.22)  && slaserdir==0){
				braco=18;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);
				}
				else if ((slasermao<=0.22 && slasermao>0.215)){
				braco=19;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);
				}
				else if ((slaserdir > 0.1 && sladodir > 0.1 && sladoesq > 0.1) && slasermao < 0.215){
				braco=20;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);
				alturaesfera=1;
				}
				ros::spinOnce();
		        	loop_rate.sleep();
				ROS_INFO("posicao braco>>%f\n",braco);
				//ROS_INFO("dist objeto>>%f\n",distobjeto);
				ROS_INFO("altura braco braco>>%f\n",slasermao);
				ROS_INFO("laser central palma da mao>>%f\n",slaserdir);
				ROS_INFO("laser direito palma da mao>>%f\n",sladodir);
				ROS_INFO("laser esquerdo palma da mao>>%f\n",sladoesq);
			
			}

			while((alturaesfera == 1 && slaserdir<0.37)|| slasermao<0.9){	
				
				braco=20;
				motor=0;
			        msg.linear.x = 0;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);	
				ros::spinOnce();
		        	loop_rate.sleep();
				if (slaserdir > 0.36){
       				alturaesfera = 2;
				ros::spinOnce();}
				ROS_INFO("posicao braco>>%f\n",braco);
				ROS_INFO("altura braco braco>>%f\n",slasermao);
				ROS_INFO("laser central palma da mao>>%f\n",slaserdir);
			}
			
			while((alturaesfera == 2) && (slaserdir > 0.01 && sladodir > 0.01 && sladoesq > 0.01)){	
				
				braco=0;
				motor=6;
			        msg.linear.x = -0.1;
			        msg.angular.z = 0;
				msgmotor.data=motor;
				msgbraco.data=braco;
		       		pub.publish(msg);
				pubbraco.publish(msgbraco);
				pubmotor.publish(msgmotor);	
				ros::spinOnce();
		        	loop_rate.sleep();	
				if (slaserdir < 0.086){
       				alturaesfera = 3;
				ros::spinOnce();}
				ROS_INFO("laser central palma da mao>>%f\n",slaserdir);
				//ROS_INFO("altura braco braco>>%f\n",slasermao);
			}

			


			while(alturaesfera == 3 && slaserdir < 0.09){ 
				braco=21;	
				motor=6;
    		        	msg.linear.x = -0.05;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararbraco=pararbraco+1;
				difsensor = abs(sladodir-sladoesq);
				if (slaserdir < 0.035){
       				alturaesfera = 4;
				ros::spinOnce();}
				ros::spinOnce();
			        loop_rate.sleep();
				//laseralterado=0;
			        //ROS_INFO("parando braco>>%f\n", pararbraco);
			       ROS_INFO("valor do sensor laser>>%f\n",slaserdir);
				//ROS_INFO("diferenca entre lado dir e esq>>%f\n", difsensor);
			        //ROS_INFO("altura do braco>>%f\n", slasermao);

			}

			while(fechargarra3<100){ 
				
				controlgarra = abs(slaserhand2 - slaserhand3);

				
				/*if (slaserhand1<0.18 && slaserhand1<0.12){
				braco=0;
				}
				if (slaserhand2>slaserhand3 && controlgarra > 0.02){
				braco=15;
				}
				if (controlgarra <= 0.02  && controlgarra > 0.012){
				braco=13;
					
				}
				if ((slaserhand1 > 0.015 && slaserhand2 > 0.015 && slaserhand3 > 0.015) && (controlgarra <= 0.012) ){
				braco=3;
				}
				if ((slaserhand1 > 0.015 && slaserhand2 > 0.015 && slaserhand3 > 0.015) || (controlgarra <= 0.01) ){
				braco=13;
				}
				else{
				braco=3;
				}*/				
				braco=14;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				fechargarra3=fechargarra3+1;
				ligarbraco=0;
				alturadoobjeto1=slasermao;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("controle da garra>>%f\n", controlgarra);
			        ROS_INFO("sensor hand1>>%f\n", slaserhand1);
			        ROS_INFO("sensor hand2>>%f\n", slaserhand2);
			        ROS_INFO("sensor hand3>>%f\n", slaserhand3);
			        ROS_INFO("valor do braco>>%f\n", braco);
			        //ROS_INFO("altura do objeto>>%f\n", alturadoobjeto1);
				alturadoobjeto1=slasermao;

			}

			


			while ( (fechargarra3<200 ) ){
			braco=3;
			motor=0;
			fechargarra3 = fechargarra3+1;
		        msg.linear.x = 0;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("fechar garra>>%f\n", braco);
			ROS_INFO("levantar braco>>%f\n", ligarbraco);

			}

			while ( (fechargarra3<300 ) ){
			braco=14;
			motor=0;
			fechargarra3 = fechargarra3+1;
		        msg.linear.x = 0;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("fechar garra>>%f\n", braco);
			//ROS_INFO("robo para tras>>%f\n", distobjeto);
			//ROS_INFO("levantar braco>>%f\n", ligarbraco);

			}

			while ( (distobjeto < 0.8 || ligarbraco3<350 ) ){
			braco=22;
			motor=6;
			ligarbraco3 = ligarbraco3+1;
		        msg.linear.x = 0.1;
		        msg.angular.z = 0.5;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			pararbraco=0;
			ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("levantar braco>>%f\n", ligarbraco);

			}

			while(pararbraco3<50){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararbraco3=pararbraco3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando braco>>%f\n", pararbraco3);
			}

// Controle da orientacao e posicao para deslocamento do robo ate o ponto B

alterarpontox=0;
abrirgarra3=0;
pararrobo3=0;
baixarbraco3=0;
bracoposinicial3=0;

pontoby=-1.7;

 	while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos) ) { //& ultraalterado

		 angulo = atan2(pontoby-feedbacky,pontobx-feedbackx);

		Erro_dist =  sqrt(pow(pontobx-feedbackx,2)+pow(pontoby-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 		

	if (((sens < 1.5 && sens > 0.1)  ||  (sensesq < 1.5 && sensesq > 0.1 ) ||  (sensdir < 1.5 && sensdir > 0.1 )) && alterarpontox==0){        //se tiver um objeto no local
	
		pontobx=pontobx+0.25;
		alterarpontox=1;

        }
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        


				if ((Ultrassom[0] < 0.05)||(Ultrassom[1] < 0.05)||(Ultrassom[2] < 0.15)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		//braco=0;
		pararrobo3=0;

         	//ROS_INFO("theta>>%f,Erro>>%f",feedbacktheta,erroorie);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

			
		        msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

			while(pararrobo3<10){ 
				braco=0;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo3=pararrobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo);
			}

			alturadoobjeto3=alturadoobjeto3+0.003;


			while ( (slasermao>=alturadoobjeto3 && pararrobo3==10 ) ){
			braco=26;// estava 5
			motor=5;
			baixarbraco3 = baixarbraco3+1;
		        msg.linear.x = 0.05;
		        msg.angular.z = 0;
			msgbraco.data=braco;
			msgbp1.data=250;
			msgmotor.data=motor;
	       		pub.publish(msg);
			pubbraco.publish(msgbraco);
			pubmotor.publish(msgmotor);
   			pubbp1.publish(msgbp1);
			ros::spinOnce();
		        loop_rate.sleep();
			//pararbraco=0;
			//ROS_INFO("robo para tras>>%f\n", distobjeto);
			ROS_INFO("baixar braco>>%f\n", baixarbraco);

			}
// colocado novo
			while(abrirgarra3 < 200){ 
				braco=14;	//estava 6
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra3=abrirgarra3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra3);
				ROS_INFO("braco>>%f\n", braco);
			}

			while(abrirgarra3 < 400){ 
				braco=23;	//estava 6
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra3=abrirgarra3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra3);
				ROS_INFO("braco>>%f\n", braco);
			}

// colocado novo
			while(abrirgarra3 < 500){ 
				braco=24;	//estava 6
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra3=abrirgarra3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra3);
				ROS_INFO("braco>>%f\n", braco);
			}

// colocado novo
			while(abrirgarra3 < 700){ 
				braco=25;	//estava 6
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra3=abrirgarra3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra);
				ROS_INFO("braco>>%f\n", braco);
			}

			while(distobjeto < 0.9){ 
				braco=25;	//estava 6
				motor=6;
    		        	msg.linear.x = 0.1;
		        	msg.angular.z = 0.1;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				abrirgarra=abrirgarra+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e robo para tras>>%f\n", abrirgarra);
				ROS_INFO("robo para tras>>%f\n", distobjeto);
			}


			while(bracoposinicial3<20){ 
				braco=7;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				bracoposinicial3=bracoposinicial3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("abrir garra e posicao inicial>>%f\n", bracoposinicial3);
			        //ROS_INFO("mensagem para braco>>%f\n", msgbraco);
			}

				while(pararrobo3>10 && pararrobo3<50){ 
				braco=7;	
				motor=0;
    		        	msg.linear.x = 0;
		        	msg.angular.z = 0;
				msgbraco.data=braco;
				msgmotor.data=motor;
	       			pub.publish(msg);
				pubmotor.publish(msgmotor);
    				pubbraco.publish(msgbraco);
				pararrobo3=pararrobo3+1;
				ros::spinOnce();
			        loop_rate.sleep();
			        ROS_INFO("parando robo>>%f\n", pararrobo3);
			}

//Fim do robo pegando e levando um copo do ponto A ate o ponto B
float posicaofinal=0;
    cout << "Voce deseja pegar outro objeto? Digite 1 para SIM e 2 para nao?\nX>>";
    cin >> posdesejada[0]; //ARMAZENA A COORDENADA X DESEJADA

	opcao = posdesejada[0];
	if (opcao==2){
		pontoax=0;
		pontoay=0;
	Erro_dist=99;
	Erro_orie=99;
	posicaofinal=1;
	}

	if (opcao==1){
		pontoax=4;
		pontoay=1.35;
	Erro_dist=99;
	Erro_orie=99;
	posicaofinal=1;
	}

     if (sensesqext < 0.05 ||  sensesq < 0.05  ||  sensdir < 0.05 ||  sensdirext < 0.05){
       ultraalterado = 1;
	ros::spinOnce();
        }

	
 	
    // Controle da orientacao e posicao

while ((abs(Erro_orie) > tolerance_orie || Erro_dist > tolerance_pos)&& posicaofinal>0.6 ) { //& ultraalterado

		 angulo = atan2(pontoay-feedbacky,pontoax-feedbackx);

		Erro_dist =  sqrt(pow(pontoax-feedbackx,2)+pow(pontoay-feedbacky,2));       

		Erro_orie = (angulo-feedbacktheta); 	

	if ((sensesq < 1 && sensesq > 0.1 ) && posicaoinicial==1){
        posicaofinal = 0.5;
	}	

	if (posicaoinicial==0){
        ultraalterado = 1;
	}	
   
	//ROS_INFO("ultraalterado>>%f\n", ultraalterado);

      double pi = 2*acos(0.0);
      if(Erro_orie > pi){
      Erro_orie = Erro_orie - 2*pi;
      }
      
      if(Erro_orie <= -pi){
      Erro_orie = Erro_orie + 2*pi;
      }

 float Ultrassom[3];
      
      if (ultrasc < 0.05){
        ultrasc = 0.5;
        }
        
        if (ultrase < 0.05){
        ultrase = 0.5;
        }
        
        if (ultrasd < 0.05){
        ultrasd = 0.5;
        }

            				
      Ultrassom[0] = ultrase;				
      Ultrassom[1] = ultrasc;
      Ultrassom[2] = ultrasd;
  
      float Sensores[3];        
      				
      Sensores[0] = min(sensesq, sensesqext);				
      Sensores[1] = sens;
      Sensores[2] = min(sensdir, sensdirext);  				        

//	ROS_INFO("Sensor1>>%f\n", Sensores[0]);
//	ROS_INFO("Sensor2>>%f\n", Sensores[1]);
//	ROS_INFO("Sensor3>>%f\n", Sensores[2]);
    			
			        

				if ((Ultrassom[0] < 0.3)||(Ultrassom[1] < 0.3)||(Ultrassom[2] < 0.3)){
					
    			msg.linear.x = fc_obs.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
    			msg.angular.z = fc_obs1.make_inference(Ultrassom[0], Ultrassom[1], Ultrassom[2]);
				}
				else{
				
 						msg.linear.x = fc1.make_inference(Erro_dist, Erro_orie);
        					msg.angular.z =  fc2.make_inference(Erro_dist, Erro_orie);
					}

		braco=0;

         	ROS_INFO("valor sensor esq1>>%f,",sensesq);
         	ROS_INFO("valor sensor esqext>>%f,",sensesqext);

        	pub.publish(msg);
		pubbraco.publish(msgbraco);
		pubmotor.publish(msgmotor);

        	ros::spinOnce();

        	loop_rate.sleep();
	}

		
			msg.linear.x = 0;
		        msg.angular.z = 0;
	       		pub.publish(msg);
			braco=0;
			motor=0;
			msgbraco.data=braco;
			msgmotor.data=motor;
			pubbraco.publish(msgbraco);
		        pubmotor.publish(msgmotor);
			pararbraco=0;
			ligarbraco=0;
			

    ROS_WARN("angulo>>%f\n", angulo);
    ROS_WARN("posicao_X>>%f\n", feedbackx);
    ROS_WARN("posicao_Y>>%f\n", feedbacky);
    ROS_WARN("...Orientacao alcancada...");
    ROS_WARN("...Posicao alcancada...");

		if(opcao==2){
			  return 0;
		}
		

  }
  return 0;
}