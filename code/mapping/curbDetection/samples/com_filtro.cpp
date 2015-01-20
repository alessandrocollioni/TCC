/******************************************************************************/
/*                          UNIVERSIDADE DE SAO PAULO                         */
/*             INSTITUTO DE CIENCIAS MATEMATICAS E DE COMPUTACAO              */
/*----------------------------------------------------------------------------*/
/* AUTOR: HEITOR DE FREITAS VIEIRA                           DATA: 19/05/2011 */
/*----------------------------------------------------------------------------*/
/* Para compilar:                                                             */
/* g++ -Wall ARQUIVO.cpp -o ARQUIVO                                           */
/*----------------------------------------------------------------------------*/
/* TESTADO COM:                                                               */
/*           - UBUNTU 10.10                                                   */
/*----------------------------------------------------------------------------*/
/* DESCRICAO DO PROGRAMA:                                                     */
/*     Exemplo de uso do detector de guias com o uso de filtro.               */
/******************************************************************************/



/******************************************************************************/
/*                              BIBLIOTECAS                                   */
/******************************************************************************/
#include <stdio.h>
#include <libplayerc/playerc.h>
#include "curbDetection.h"

/******************************************************************************/
/*                           ROTINA PRINCIPAL                                 */
/******************************************************************************/
int main(int argc, const char *argv[]) {

    /* Variáveis do player */
    playerc_client_t *client;
    playerc_laser_t *laser;
    
    /* Conecta ao player */
    client = playerc_client_create(NULL, "localhost", 6665);
    if (playerc_client_connect(client) != 0)
        return -1;

    /* Cria a conexão com o laser do player */
    laser = playerc_laser_create(client, 1);
    if (playerc_laser_subscribe(laser, PLAYERC_OPEN_MODE))
        return -1;
        
    /* Lê as informações do cliente */
    playerc_client_read(client);
    playerc_client_read(client);
    playerc_client_read(client);
    playerc_client_read(client);
    
    /* Inicia as variáveis para a detecção de guia */
    cdInit(); //Necessário somente se for utilizar o filtro nas leituras das guias
            
    /* Detecta guias com o uso de filtro */
    while(1){
    
        /* Procura pelas guias direita e esquerda ao mesmo tempo */
        cdFindCurbs(laser); 

        /* Aplica o filtro */
        cdMeanFilter();

        /* Caso não tenha guia por mais de 3 leituras o cdGetMeanDistance vai retornar 0. 
           Com isso pode-se saber se tem guia ou não. No caso do uso com filtro, 
           deve-se esperar pelo menos 3 leituras do laser pra poder usar os resultados.
         */
         
        printf("A distância entre o centro do laser e a guia direita é: %.3fm", cdGetMeanDistance(RIGHT));
        printf("A distância entre o centro do laser e a guia esquerda é: %.3fm", cdGetMeanDistance(LEFT));            
    }

    /* Desaloca os recursos do player */
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);

    return 0;
}
