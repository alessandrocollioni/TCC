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
/*     Exemplo de uso do detector de guias sem o uso de filtro.               */
/******************************************************************************/



/******************************************************************************/
/*                              BIBLIOTECAS                                   */
/******************************************************************************/
#include <stdio.h>
#include "curbDetection.h"
#include <libplayerc/playerc.h>


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
            
    /* Detecta guias sem o uso de filtro */
    while(1){
    
        /* Adquire dados do cliente */
        playerc_client_read(client);

        /* Procura pelas guias direita e esquerda ao mesmo tempo */
        cdFindCurbs(laser); //Neste caso será utilizado o valor padrão de altura da guia (7cm)

        /* Verifica se encontrou a guia direita ou não */
        if(cdFoundCurb(RIGHT)){
            printf("A distância entre o centro do laser e a guia direita é: %.3fm", cdGetCurbDistance(RIGHT));
        }
        else {
            printf("Guia direita não encontrada\n");
        }

        /* Verifica se encontrou a guia esquerda ou não */
        if(cdFoundCurb(LEFT)){
            printf("A distância entre o centro do laser e a guia esquerda é: %.3fm", cdGetCurbDistance(LEFT));
        }
        else {
            printf("Guia esquerda não encontrada\n");
        }
    }
    
    /* Desaloca os recursos do player */
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);
    
    return 0;
}
