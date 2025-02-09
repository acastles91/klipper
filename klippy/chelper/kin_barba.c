// Barba kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

#ifdef AB_COORDSPACE

static double
barba_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).a;
}

static double
barba_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).b;
}


struct stepper_kinematics * __visible
barba_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'a') {
        sk->calc_position_cb = barba_stepper_a_calc_position;
        sk->active_flags = AF_A;
    } else if (axis == 'b') {
        sk->calc_position_cb = barba_stepper_b_calc_position;
        sk->active_flags = AF_B;
    }
        return sk;
}
#endif