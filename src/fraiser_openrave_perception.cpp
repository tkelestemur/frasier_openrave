#include <frasier_openrave/frasier_openrave.h>


void FRASIEROpenRAVE::addBoxCollObj(OpenRAVE::Vector& size, OpenRAVE::Transform& pose, std::string& obj_name) {
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());

    OpenRAVE::KinBodyPtr obj_body = OpenRAVE::RaveCreateKinBody(env_);
    obj_body->SetName(obj_name);

    OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

    OpenRAVE::KinBody::GeometryInfo body;
    body._bVisible = true;
    body._type = OpenRAVE::GeometryType::GT_Box;
    body._vGeomData = OpenRAVE::Vector(size[0]/2, size[1]/2, size[2]/2);
//    body._t = hsr_pose * obj_pose;

    std::list<OpenRAVE::KinBody::GeometryInfo> geoms;
    geoms.push_back(body);

    obj_body->InitFromGeometries(geoms);

    obj_body->SetTransform(hsr_pose * pose);

    env_->Add(obj_body);

}



void FRASIEROpenRAVE::removeCollisionObj(std::string& obj_name) {
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
    bool result = env_->Remove(env_->GetKinBody(obj_name));
}