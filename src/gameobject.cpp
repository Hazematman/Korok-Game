#include "gameobject.hpp"

void GameObject::draw(float dt)
{
#if 0
    if(phys != NULL)
    {
        btTransform trans = phys->body->getWorldTransform();
        btVector3 pos = trans.getOrigin();
        btQuaternion rot = trans.getRotation();

        if(model->model_def != NULL)
        {
            float x_size = (model->model_def->max_x - model->model_def->min_x)*model->scale.x;
            float y_size = (model->model_def->max_y - model->model_def->min_y)*model->scale.y;
            float z_size = (model->model_def->max_z - model->model_def->min_z)*model->scale.z;
            float x_origin = model->scale.x*model->model_def->min_x + (x_size/2);
            float y_origin = model->scale.y*model->model_def->min_y + (y_size/2);
            float z_origin = model->scale.z*model->model_def->min_z + (z_size/2);
            pos -= btVector3(x_origin, y_origin, z_origin);
        }

        model->position = glm::vec3(pos[0], pos[1], pos[2]);
        model->rotation = glm::quat(rot[3], rot[0], rot[1], rot[2]);   
    }
#endif

    if(model != NULL)
    {
        model->draw(dt);
    }
}
