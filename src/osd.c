/* vim: set tabstop=3 expandtab:
**
** This file is in the public domain.
**
** osd.c
**
** $Id: osd.c,v 1.2 2001/04/27 14:37:11 neil Exp $
**
*/

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
       
#include "noftypes.h"
#include "nofconfig.h"
#include "log.h"
#include "osd.h"
#include "nofrendo.h"

#include "version.h"

extern char* selectedRomFilename;
char configfilename[]="na";

/* This is os-specific part of main() */
int osd_main(int argc, char *argv[])
{
   config.filename = configfilename;

   printf("Starting main loop with file: %s\n", selectedRomFilename);
   return main_loop(selectedRomFilename, system_autodetect);
}

/* File system interface */
void osd_fullname(char *fullname, const char *shortname)
{
   strncpy(fullname, shortname, PATH_MAX);
}

/* This gives filenames for storage of saves; DESTRUCTIVE */
char *osd_newextension(char *name, char *ext)
{
   int len=strlen(name);
   name[len-4] = ext[0];
   name[len-3] = ext[1];
   name[len-2] = ext[2];
   name[len-1] = ext[3];
   
   return name;
}

/* This gives filenames for storage of PCX snapshots */
int osd_makesnapname(char *filename, int len)
{
   return -1;
}
