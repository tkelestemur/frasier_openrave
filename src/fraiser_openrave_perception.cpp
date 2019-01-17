#include <frasier_openrave/frasier_openrave.h>


void FRASIEROpenRAVE::addBoxCollObj(OpenRAVE::Vector &size, OpenRAVE::Transform &pose,
                                    std::string &obj_name, bool collision) {

    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());

    OpenRAVE::KinBodyPtr obj_body = OpenRAVE::RaveCreateKinBody(env_);
    obj_body->SetName(obj_name);

    OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

    OpenRAVE::KinBody::GeometryInfo body;
    body._bVisible = true;
    body._type = OpenRAVE::GeometryType::GT_Box;
    body._vGeomData = OpenRAVE::Vector(size[0] / 2, size[1] / 2, size[2] / 2);
    body._vDiffuseColor = OpenRAVE::Vector(0, 0, 1);

    std::list<OpenRAVE::KinBody::GeometryInfo> geoms;
    geoms.push_back(body);

    obj_body->InitFromGeometries(geoms);
    obj_body->SetTransform(hsr_pose * pose);

    if (!collision)
        obj_body->Enable(false);

    try {
        env_->Add(obj_body);
        std::cout << "RAVE: added box collision object!" << std::endl;
    }
    catch (const OpenRAVE::openrave_exception &or_except) {
        std::cout << "RAVE: exception : " << or_except.message() << std::endl;
    }


}

void FRASIEROpenRAVE::addCylinderCollObj(OpenRAVE::Vector &size, OpenRAVE::Transform &pose,
                                         std::string &obj_name, bool collision) {

    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());

    OpenRAVE::KinBodyPtr obj_body = OpenRAVE::RaveCreateKinBody(env_);
    obj_body->SetName(obj_name);

    OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

    OpenRAVE::KinBody::GeometryInfo body;
    body._bVisible = true;
    body._type = OpenRAVE::GeometryType::GT_Cylinder;
    body._vGeomData = OpenRAVE::Vector(size[0] / 2, size[1] / 2, 0.0);
    body._vDiffuseColor = OpenRAVE::Vector(1, 0, 0);
    body._fTransparency = 0.2;


    std::list<OpenRAVE::KinBody::GeometryInfo> geoms;
    geoms.push_back(body);

    obj_body->InitFromGeometries(geoms);
    obj_body->SetTransform(hsr_pose * pose);

    if (!collision)
        obj_body->Enable(false);

    try {
        env_->Add(obj_body);
        std::cout << "RAVE: added cylinder collision object!" << std::endl;
    }
    catch (const OpenRAVE::openrave_exception &or_except) {
        std::cout << "RAVE: exception : " << or_except.message() << std::endl;
    }

}

//void FRASIEROpenRAVE::addMeshCollObj(pcl_msgs::PolygonMesh &mesh, std::string& obj_name) {
//
////  HACDInterface hi = HACDInterface();
////  pcl::PolygonMesh triangles;
////  pcl_conversions::toPCL(mesh, triangles);
////  std::vector<pcl::PolygonMesh::Ptr> convex_hulls = hi.ConvexDecompHACD(triangles, 25);
//
//  OpenRAVE::TriMesh trimesh;
////  int i = 0;
////  for(const auto hull : convex_hulls){
////    pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
////    pcl::fromPCLPointCloud2(hull->cloud, mesh_cloud);
////    std::string body_name = obj_name + std::to_string(i);
////    for(auto point : mesh_cloud.points){
////      OpenRAVE::Vector p;
////      p.x = point.x;
////      p.y = point.y;
////      p.z = point.z;
////      p.w = 1;
////
////      trimesh.vertices.push_back(p);
////    }
////
////    for(auto index : hull->polygons){
////      trimesh.indices.push_back(index.vertices[0]);
////      trimesh.indices.push_back(index.vertices[1]);
////      trimesh.indices.push_back(index.vertices[2]);
////    }
////
////    {
////      OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
////      OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env_);
////      body->InitFromTrimesh(trimesh, true);
////      body->SetName(body_name);
////      try {
////        std::cout << "adding mesh # of indicies: " << trimesh.vertices.size() <<  std::endl;
////        env_->Add(body);
////      }
////      catch (const OpenRAVE::openrave_exception &or_except)
////      {
////        std::cout << "Caught: Open rave exception "<< or_except.message() << std::endl;
////      }
////    }
////
////    i++;
////  }
//
//    pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
//    pcl::fromROSMsg(mesh.cloud, mesh_cloud);
//
//    for(auto point : mesh_cloud.points){
//      OpenRAVE::Vector p;
//      p.x = point.x;
//      p.y = point.y;
//      p.z = point.z;
//      p.w = 1;
//
//      trimesh.vertices.push_back(p);
//    }
//
//    for(auto index : mesh.polygons){
//      trimesh.indices.push_back(index.vertices[0]);
//      trimesh.indices.push_back(index.vertices[1]);
//      trimesh.indices.push_back(index.vertices[2]);
//    }
//
//    {
//      OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
//      OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env_);
//      body->InitFromTrimesh(trimesh, true);
//      body->SetName(obj_name);
//      try {
//        std::cout << "adding mesh # of indicies: " << trimesh.vertices.size() <<  std::endl;
//        env_->Add(body);
//      }
//      catch (const OpenRAVE::openrave_exception &or_except)
//      {
//        std::cout << "Caught: Open rave exception "<< or_except.message() << std::endl;
//      }
//    }
//
//
//}

void FRASIEROpenRAVE::addMeshCollObj(pcl_msgs::PolygonMesh &mesh, std::string &obj_name) {

    std::cout << "Begin operating with triangles, size: " << mesh.polygons.size() << "." << std::endl;
    pcl::PointCloud<pcl::PointXYZ> meshCloud;
    fromROSMsg(mesh.cloud, meshCloud);
  OpenRAVE::TriMesh raveMesh;

    for (size_t triangle_index = 0; triangle_index < meshCloud.size(); triangle_index++) {
        // vert 0
        raveMesh.indices.push_back(3 * triangle_index);
        size_t point0_index = mesh.polygons[triangle_index].vertices[0];
        raveMesh.vertices.push_back(OpenRAVE::Vector(meshCloud.points[point0_index].x,
                                                     meshCloud.points[point0_index].y,
                                                     meshCloud.points[point0_index].z));
        // vert 1
        raveMesh.indices.push_back(1 + 3 * triangle_index);
        size_t point1_index = mesh.polygons[triangle_index].vertices[1];
        raveMesh.vertices.push_back(OpenRAVE::Vector(meshCloud.points[point1_index].x,
                                                     meshCloud.points[point1_index].y,
                                                     meshCloud.points[point1_index].z));
        // vert 2
        raveMesh.indices.push_back(2 + 3 * triangle_index);
        size_t point2_index = mesh.polygons[triangle_index].vertices[2];
        raveMesh.vertices.push_back(OpenRAVE::Vector(meshCloud.points[point2_index].x,
                                                     meshCloud.points[point2_index].y,
                                                     meshCloud.points[point2_index].z));
    }
    { // lock the environment!
        OpenRAVE::EnvironmentMutex::scoped_lock lock(env_->GetMutex());
        OpenRAVE::KinBodyPtr meshKinBodyPtr = RaveCreateKinBody(env_, "");
        meshKinBodyPtr->SetName("m");
        meshKinBodyPtr->InitFromTrimesh(raveMesh, true);
        //-- Add new
//        env_->Add(meshKinBodyPtr, true);
//        drawMesh(raveMesh);
    }
}


void FRASIEROpenRAVE::removeCollisionObj(std::string &obj_name) {
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
    bool result = env_->Remove(env_->GetKinBody(obj_name));
}

void FRASIEROpenRAVE::removeTableObjects() {
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());

    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto body : bodies) {
        std::string body_name = body->GetName();

        if (body_name.substr(0, 6) == "ground") {
            env_->Remove(body);
        }

    }
}

void FRASIEROpenRAVE::getObjectPose(OpenRAVE::Transform &pose, std::string &obj_name) {

    OpenRAVE::KinBodyPtr body = env_->GetKinBody(obj_name);

    pose = body->GetTransform();

}

void FRASIEROpenRAVE::grabObject(std::string &obj_name) {
    OpenRAVE::KinBodyPtr grabbed_object = env_->GetKinBody(obj_name);
    hsr_->Grab(grabbed_object);
}

void FRASIEROpenRAVE::releaseObject(std::string &obj_name) {
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
    OpenRAVE::KinBodyPtr released_object = env_->GetKinBody(obj_name);
    hsr_->Release(released_object);
    std::string new_name = "released_" + obj_name;
    released_object->SetName(new_name);
}


