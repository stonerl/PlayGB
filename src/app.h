//
//  app.h
//  PlayGB
//
//  Created by Matteo D'Ignazio on 14/05/22.
//

#ifndef app_h
#define app_h

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "pd_api.h"
#include "scene.h"
#include "utility.h"

#if defined(TARGET_PLAYDATE) && !defined(TARGET_SIMULATOR) && !defined(TARGET_DEVICE)
    #define TARGET_DEVICE 1
#endif

#if !defined(TARGET_DEVICE) && defined(DTCM_ALLOC)
    #undef DTCM_ALLOC
#endif

// TODO: is this safe? should we lower it?
#define PLAYDATE_STACK_SIZE 0x2000

typedef struct PGB_Application {
    float dt;
    float crankChange;
    PGB_Scene *scene;
    PGB_Scene *pendingScene;
    LCDFont *bodyFont;
    LCDFont *titleFont;
    LCDFont *subheadFont;
    LCDFont *labelFont;
    LCDBitmapTable *selectorBitmapTable;
    LCDBitmap *startSelectBitmap;
    SoundSource *soundSource;
} PGB_Application;

extern PGB_Application *PGB_App;

void PGB_init(void);
void PGB_update(float dt);
void PGB_present(PGB_Scene *scene);
void PGB_quit(void);

// allocates in DTCM region (if enabled).
// note, there is no associated free.
void* dtcm_alloc(size_t size);

#define PLAYDATE_ROW_STRIDE 52

#endif /* app_h */
