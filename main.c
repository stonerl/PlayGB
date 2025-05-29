//
//  main.c
//  PlayGB
//
//  Created by Matteo D'Ignazio on 14/05/22.
//

#include <stdio.h>
#include "pd_api.h"
#include "app.h"

#ifdef _WINDLL
#define DllExport __declspec(dllexport)
#else
#define DllExport
#endif

static int update(void* userdata);

#ifdef DTCM_ALLOC
// low address that's within stack region,
// can allocate global variables here
static void* dtcm_mempool;
static uint32_t* dtcm_low_canary_addr = NULL;
#define DTCM_CANARY 0xDE0DCA94
#endif

void* dtcm_alloc(size_t size)
{
#ifdef DTCM_ALLOC
    void* tmp = dtcm_mempool;
    *(uint32_t*)dtcm_mempool = 0;
    dtcm_mempool = (void*)(size + (uintptr_t)dtcm_mempool);
    // high canary
    *(uint32_t*)dtcm_mempool = DTCM_CANARY;
    return tmp;
#else
    return playdate->system->realloc(NULL, size);
#endif
}

static void dtcm_init(void* addr)
{
    playdate->system->logToConsole("Top of stack: %p\n", addr);
    
    #ifdef DTCM_ALLOC
    dtcm_mempool = addr;
    *(uint32_t*)dtcm_mempool = DTCM_CANARY;
    dtcm_low_canary_addr = (uint32_t*)dtcm_alloc(sizeof(uint32_t));
    *dtcm_low_canary_addr = DTCM_CANARY;
    #endif
}

static bool dtcm_verify(void)
{
#ifdef DTCM_ALLOC
    if (dtcm_low_canary_addr)
    {
        if (*dtcm_low_canary_addr != DTCM_CANARY)
        {
            playdate->system->error("DTCM low canary broken (decrease PLAYDATE_STACK_SIZE?)");
            return false;
        }
        if (*(uint32_t*)dtcm_mempool != DTCM_CANARY)
        {
            playdate->system->error("DTCM high canary broken (stack overflow?)");
            return false;
        } 
    }
#endif
    return true;
}

DllExport int eventHandler(PlaydateAPI *pd, PDSystemEvent event, uint32_t arg)
{
    if (!dtcm_verify()) return 0;
    
    if(event == kEventInit)
    {
        playdate = pd;
        
        dtcm_init(__builtin_frame_address(0) - PLAYDATE_STACK_SIZE);
        
        PGB_init();
        
        pd->system->setUpdateCallback(update, pd);
    }
    else if (event == kEventTerminate)
    {
        PGB_quit();
    }
    
    return 0;
}

int update(void* userdata)
{
    PlaydateAPI *pd = userdata;
    
    float dt = pd->system->getElapsedTime();
    pd->system->resetElapsedTime();
    
    PGB_update(dt);
    
    return 1;
}
