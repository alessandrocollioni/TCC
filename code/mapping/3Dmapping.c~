
/******************************************************************************/
/*                          UNIVERSIDADE DE SAO PAULO                         */
/*             INSTITUTO DE CIENCIAS MATEMATICAS E DE COMPUTACAO              */
/*----------------------------------------------------------------------------*/
/* DESENVOLVIDO POR: HEITOR DE FREITAS VIEIRA                DATA: 07/03/2009 */
/*                                                                            */
/* PODE SER ALTERADO E DISTIBUIDO LIVREMENTE, DESDE QUE MENCIONADO O AUTOR.   */
/*----------------------------------------------------------------------------*/
/* Para compilar:                                                             */
/* gcc -Wall terreno3D_RC.c -o terreno3D_RC -I/usr/local/include/player-2.1 -L/usr/local/lib -lplayerc -lm -lplayerxdr -lplayererror
/******************************************************************************/

#include <stdio.h>
#include <libplayerc/playerc.h>
#include <math.h>

#define PI 3.14159265
#define AlturaRobo  0.3            //Coloque aqui a altura do robo (em metros)
#define DistRodaLaser 0.1          //Distancia entre o laser e a roda do robo (em metros)

/******************************************************************************/
/*                  CONVERTE ANGULOS EM GRAUS PARA RADIANOS                   */
/******************************************************************************/
float Graus2Rad (int graus) {
   float temp = ((graus*PI)/180);
   return temp;
}

/******************************************************************************/
/*                            ROTINA PRINCIPAL                                */
/******************************************************************************/
int main(int argc, const char **argv) {
    //Variaveis
    FILE *fp;
    int FeixeLaser, aux;
    float PosGlobX,PosGlobY,PosGlobZ,AngGlob, Distancia, AnguloLaser;
    playerc_client_t *client;
    playerc_laser_t *laser;
    playerc_position2d_t *position2d;

    client = playerc_client_create(NULL, "localhost", 6665);
    if (playerc_client_connect(client) != 0)
        return -1;

    laser = playerc_laser_create(client, 0);
    if (playerc_laser_subscribe(laser, PLAYERC_OPEN_MODE))
        return -1;

    position2d = playerc_position2d_create(client, 0);
    if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    if (playerc_client_datamode (client, PLAYERC_DATAMODE_PULL) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
     }

    if (playerc_client_set_replace_rule (client, -1, -1, PLAYER_MSGTYPE_DATA, -1, 1) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
     }

    playerc_position2d_enable(position2d, 1);  // Liga os motores
    playerc_position2d_set_odom(position2d, 0, 0, 0);  // Zera o odômetro


    // abre o arquivo para gravação
    if ((fp = fopen("Mapa3D.wrl","w"))==NULL) {
      printf("Erro na abertura do arquivo VRML!\n");
      return (1);
    }

    //Prepara o cabecalho do VRML
    fprintf(fp, "#VRML V2.0 utf8\n");
    fprintf(fp, "Shape { appearance Appearance\n");
    fprintf(fp,"     { material Material {emissiveColor 1 1 1 } }\n");
    fprintf(fp,"          geometry PointSet { coord Coordinate {\n");
    fprintf(fp,"                point [ \n");

    printf("Qual e o angulo entre o laser e o chao (em graus)? ");
    scanf("%d\n", &aux);
    AnguloLaser = Graus2Rad(aux);

    while(1){
        playerc_client_read(client);
        //playerc_position2d_set_cmd_vel(position2d, 0.2, 0, 0, 1);
        playerc_client_read(client);

        for (FeixeLaser = 0; FeixeLaser <= 360; FeixeLaser++) {
            Distancia = laser->scan[FeixeLaser][0];
            if (Distancia < 8) {

                //Calcula a Coordenada Global Z
                PosGlobZ = (AlturaRobo-(sin(AnguloLaser)*cos(laser->scan[FeixeLaser][1])*Distancia));

                //Calcula a Coordenada Global X
                PosGlobX = (position2d->px+DistRodaLaser+(((cos(AnguloLaser)*cos(laser->scan[FeixeLaser][1])*Distancia)*cos(position2d->pa))-((sin(laser->scan[FeixeLaser][1])*Distancia)*sin(position2d->pa))));

                //Calcula a Coordenada Global Y
                PosGlobY = (position2d->py+(((cos(AnguloLaser)*cos(laser->scan[FeixeLaser][1])*Distancia)*sin(position2d->pa))+((sin(laser->scan[FeixeLaser][1])*Distancia)*cos(position2d->pa))));

                fprintf(fp,"%.3f %.3f %.3f\n",PosGlobX, PosGlobY,PosGlobZ);
            }
        }
    }

    //Desliga os motores do robo
    playerc_position2d_set_cmd_vel(position2d, 0, 0, 0, 1);

    //Desconecta o player
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);

    //Fecha o arquivo VRML
    fprintf(fp,"                      ]\n");
    fprintf(fp,"            }\n");
    fprintf(fp,"     }\n");
    fprintf(fp," }\n");
    fclose(fp);

    return 0;
}
