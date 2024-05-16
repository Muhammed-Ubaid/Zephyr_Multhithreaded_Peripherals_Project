/**
 * @brief : This file defines a macro to use for realizing singleton design pattern.
*/
#include <zephyr/spinlock.h>


#ifndef SINGLETON_H
#define SINGLETON_H

#define DEFINE_SINGLETON_INSTANCE(type)         \
static struct type *_##type##_self;             \
static struct k_spinlock _##type##_lock;        \
struct type *type_get_singleton_handle(void)    \
{   \
    k_spinlock_key_t key = k_spin_lock(&lock);  \
    if(! _##type##_self)     {                  \
    static struct type _singleton = {0};        \
    _##type##_self = &_singleton;               \
    ##type##_init(__##type##_self);             \
    }                                           \
    k_spin_unlock(&_##type##_lock, key);        \
    return _##type##_self;                      \
}    \

#endif