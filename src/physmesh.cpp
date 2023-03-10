#include "physmesh.hpp"

#if 0
PhysMesh::PhysMesh(ugfx_vertex_t *verticies, uint32_t num_verts, btDiscreteDynamicsWorld *dynWorld, 
                   btVector3 origin, uint32_t bitmask, uint32_t group)
{
    btVector3 verts[3];
    trimesh = new btTriangleMesh();

    for(uint32_t i=0; i < num_verts; i++)
    {
        uint32_t tri_index = i % 3;
        verts[tri_index] = btVector3(fixed_to_float(verticies[i].x, 5), 
                                     fixed_to_float(verticies[i].y, 5),
                                     fixed_to_float(verticies[i].z, 5));

        if(tri_index == 2)
        {
            trimesh->addTriangle(verts[0], verts[1], verts[2]);
        }
    }

    gimpact = new btGImpactMeshShape(trimesh);
    gimpact->updateBound();
    col_shape = gimpact;

    motion_state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), origin));
    btVector3 inertia;
    col_shape->calculateLocalInertia(0, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(0, motion_state, col_shape, inertia);
    body = new btRigidBody(ci);

    dynWorld->addRigidBody(body, group, bitmask);
}
#endif

PhysMesh::PhysMesh(float mass, float radius, btDiscreteDynamicsWorld *dynWorld, 
                   btVector3 origin, uint32_t bitmask, uint32_t group)
{
    col_shape = new btSphereShape(radius);
    motion_state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), origin));
    btVector3 inertia;
    col_shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motion_state, col_shape, inertia);
    body = new btRigidBody(ci);
    dynWorld->addRigidBody(body, group, bitmask);
}

PhysMesh::PhysMesh(float mass, btVector3 extents, btDiscreteDynamicsWorld *dynWorld, btVector3 origin, 
                   uint32_t bitmask, uint32_t group)
{
    col_shape = new btBoxShape(extents);
    motion_state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), origin));
    btVector3 inertia;
    col_shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motion_state, col_shape, inertia);
    body = new btRigidBody(ci);
    dynWorld->addRigidBody(body, group, bitmask);
}
