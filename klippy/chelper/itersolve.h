#ifndef ITERSOLVE_H
#define ITERSOLVE_H

#include "coordspace.h" // struct coord
#include <stdint.h> // int32_t

#ifdef XYZ_COORDSPACE
enum {
    AF_X = 1 << 0, AF_Y = 1 << 1, AF_Z = 1 << 2,
};

#endif

#ifdef AB_COORDSPACE

enum {
    AF_A = 1 << 0, AF_B = 1 << 1,
};

#endif

struct stepper_kinematics;
struct move;
typedef double (*sk_calc_callback)(struct stepper_kinematics *sk, struct move *m
                                   , double move_time);
typedef void (*sk_post_callback)(struct stepper_kinematics *sk);
struct stepper_kinematics {
    double step_dist, commanded_pos;
    struct stepcompress *sc;

    double last_flush_time, last_move_time;
    struct trapq *tq;
    int active_flags;
    double gen_steps_pre_active, gen_steps_post_active;

    sk_calc_callback calc_position_cb;
    sk_post_callback post_cb;
};

int32_t itersolve_generate_steps(struct stepper_kinematics *sk
                                 , double flush_time);
double itersolve_check_active(struct stepper_kinematics *sk, double flush_time);
int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);
void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
void itersolve_set_stepcompress(struct stepper_kinematics *sk
                                , struct stepcompress *sc, double step_dist);

#ifdef XYZ_COORDSPACE
double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
                                          , double x, double y, double z);
void itersolve_set_position(struct stepper_kinematics *sk
                            , double x, double y, double z);
#endif

#ifdef AB_COORDSPACE

double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
                                          , double a, double b);
void itersolve_set_position(struct stepper_kinematics *sk
                            , double a, double b);  
#endif

double itersolve_get_commanded_pos(struct stepper_kinematics *sk);

#endif // itersolve.h
