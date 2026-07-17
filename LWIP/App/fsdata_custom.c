/* fsdata_custom.c
 *
 * Table de fichiers statiques du httpd LwIP. Volontairement VIDE :
 * toutes les pages de l'interface web sont générées dynamiquement en RAM
 * par fs_open_custom() (voir web_ui.c). Ce fichier n'existe que pour
 * satisfaire l'inclusion « #include HTTPD_FSDATA_FILE » de fs.c.
 *
 * Généré normalement par l'outil makefsdata de LwIP ; ici écrit à la main.
 */

#include "lwip/apps/fs.h"
#include "lwip/def.h"

#define file_NULL (struct fsdata_file *) NULL

/* Aucune racine : FS_ROOT = NULL -> fs_open() délègue entièrement aux
 * custom files. */
#define FS_ROOT file_NULL
#define FS_NUMFILES 0
