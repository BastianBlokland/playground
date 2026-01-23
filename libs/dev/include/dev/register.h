#pragma once
#include "ecs/def.h"

enum {
  DevOrder_TraceQuery  = -1000,
  DevOrder_RendUpdate  = -400,
  DevOrder_TextRender  = 750,
  DevOrder_ShapeRender = 850,
};

/**
 * Register the ecs modules for the Development library.
 */
void dev_register(EcsDef*);
