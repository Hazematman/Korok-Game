#ifndef GAMEOBJECT_HPP
#define GAMEOBJECT_HPP
#include "physmesh.hpp"
#include "model.hpp"

enum GameObjectType
{
    GOBJ_PLAYER,
    GOBJ_CRYSTAL,
    GOBJ_TOTEM,
    GOBJ_MUSHROOM,
    GOBJ_CENTI,
};

class GameObject
{
public:
    GameObjectType type;
    bool can_collide;
    Model *model;
    PhysMesh *phys;
    int collision;

    void draw(float dt=0);
};

#endif
