#pragma once

namespace bio_gait {

enum class GaitType
{
    STAND, 
    STATIC_WALK,  
    TROT_WALK, 
    TROT,
    TROT_RUN,
    ROTARY_GALLOP,
    PACE, 
    BOUND, 
    TRAVERSE_GALLOP, 
    PRONK, 
    INTERMEDIATEGAIT,
    UNNATURAL,
    AMBLE,
    HOP,
};

}  // end bio_gait namespace
