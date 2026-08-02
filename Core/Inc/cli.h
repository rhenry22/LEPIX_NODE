#ifndef CLI_H
#define CLI_H

/* Console de configuration sur USART1 (DB9, 115200 8N1).
 * Reception par interruption (ring buffer), traitement dans la boucle
 * principale via CLI_Task(). Les reponses passent par printf (_write
 * emet deja sur huart1). Taper "help" pour la liste des commandes. */

void CLI_Init(void);
void CLI_Task(void);

#endif /* CLI_H */
