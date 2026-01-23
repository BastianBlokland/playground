#include "ecs/def.h"
#include "snd/register.h"

void snd_register(EcsDef* def) { ecs_register_module(def, snd_mixer_module); }
