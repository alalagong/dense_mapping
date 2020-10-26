#pragma once

#include "sys/types.h"
#include "sys/sysinfo.h"
#include <stdio.h>
#include <cstdio>
#include <inttypes.h>

typedef struct ProcessList_
{
   unsigned long long totalMem;
   unsigned long long usedMem;
   unsigned long long freeMem;
   unsigned long long sharedMem;
   unsigned long long buffersMem;
   unsigned long long cachedMem;
   unsigned long long totalSwap;
   unsigned long long usedSwap;
   unsigned long long freeSwap;
} ProcessList;

static inline void LinuxProcessList_scanMemoryInfo(ProcessList* pl)
{
   #define String_startsWith(s, match) (strncmp((s),(match),strlen(match)) == 0)

   unsigned long long swapFree = 0;
   unsigned long long shmem = 0;
   unsigned long long sreclaimable = 0;

   char PROCMEMINFOFILE[128] = "/proc/meminfo";

   FILE* file = fopen(PROCMEMINFOFILE, "r");
   if (file == NULL) {
      printf("Cannot open %s\n", PROCMEMINFOFILE);
   }
   char buffer[128];
   while (fgets(buffer, 128, file))
   {
      //printf("%s\n", buffer ) ;
      #define tryRead(label, variable) (String_startsWith(buffer, label) && sscanf(buffer + strlen(label), "%32llu kB", variable))
      switch (buffer[0]) {
      case 'M':
         if (tryRead("MemTotal:", &pl->totalMem)) {
             //cout << "MemTotal:" << pl->totalMem << "\n" ;
         }
         else if (tryRead("MemFree:", &pl->freeMem)) {}
         break;
      case 'B':
         if (tryRead("Buffers:", &pl->buffersMem)) {}
         break;
      case 'C':
         if (tryRead("Cached:", &pl->cachedMem)) {}
         break;
      case 'S':
         switch (buffer[1]) {
         case 'w':
            if (tryRead("SwapTotal:", &pl->totalSwap)) {}
            else if (tryRead("SwapFree:", &swapFree)) {}
            break;
         case 'h':
            if (tryRead("Shmem:", &shmem)) {}
            break;
         case 'R':
            if (tryRead("SReclaimable:", &sreclaimable)) {}
            break;
         }
         break;
      }
      #undef tryRead
   }
   std::fclose(file);

   //cout << pl->totalMem << " " << pl->freeMem << "\n" ;

   pl->usedMem = pl->totalMem - pl->freeMem;
   pl->cachedMem = pl->cachedMem + sreclaimable - shmem;
   pl->usedSwap = pl->totalSwap - swapFree;

   pl->usedMem -= pl->buffersMem + pl->cachedMem;
}
