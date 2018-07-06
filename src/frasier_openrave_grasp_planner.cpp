#include <frasier_openrave/frasier_openrave.h>

////////////////////////////// GRASPING //////////////////////////////

Grasp FRASIEROpenRAVE::generateGraspPose(int table_pose, std::string &obj_name) {
  Grasp grasp;
  grasp.obj_name = obj_name;
//  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();
  OpenRAVE::KinBodyPtr grasp_body = env_->GetKinBody(obj_name);

//  OpenRAVE::Transform object_pose = hsr_pose.inverse() * grasp_body->GetTransform();
//  object_pose.rot = LEFT_EEF_ROT;
//  object_pose.trans.y = object_pose.trans.y - 0.03;

  OpenRAVE::Vector object_size = grasp_body->GetLink("base")->GetGeometry(0)->GetBoxExtents();
  OpenRAVE::Transform object_pose = grasp_body->GetTransform();

  if(table_pose == FRONT_TABLE){
    std::cout << "RAVE: table is front!" << obj_name << std::endl;
    if(object_size[1]*2 < MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected side grasp for " << obj_name << std::endl;
      grasp.pose.rot = FRONT_EEF_ROT;
      grasp.pose.trans.x = object_pose.trans.x - 0.02;
      grasp.pose.trans.y = object_pose.trans.y;
      grasp.pose.trans.z = object_pose.trans.z;
    }
    else if(object_size[1]*2 >= MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected top grasp for " << obj_name << std::endl;
      if(object_size[0]*2 < MAX_FINGER_APERTURE){
        grasp.pose.rot = FRONT_TOP_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x;
        grasp.pose.trans.y = object_pose.trans.y;
        grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
      }
    }
  }
  else if(table_pose == LEFT_TABLE){
    std::cout << "RAVE: table is left!" << obj_name << std::endl;
    if(object_size[0]*2 < MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected side grasp for " << obj_name << std::endl;
      grasp.pose.rot = LEFT_EEF_ROT;
      grasp.pose.trans.x = object_pose.trans.x;
      grasp.pose.trans.y = object_pose.trans.y - 0.02;
      grasp.pose.trans.z = object_pose.trans.z;
    }
    else if(object_size[0]*2 >= MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected top grasp for " << obj_name << std::endl;
      if(object_size[1]*2 < MAX_FINGER_APERTURE){
        grasp.pose.rot = FRONT_TOP_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x;
        grasp.pose.trans.y = object_pose.trans.y;
        grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
      }
    }
  }
  else if(table_pose == RIGHT_TABLE){
    std::cout << "RAVE: table is right!" << obj_name << std::endl;
    if(object_size[0]*2 < MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected side grasp for " << obj_name << std::endl;
      grasp.pose.rot = RIGHT_EEF_ROT;
      grasp.pose.trans.x = object_pose.trans.x;
      grasp.pose.trans.y = object_pose.trans.y + 0.02;
      grasp.pose.trans.z = object_pose.trans.z;
    }
    else if(object_size[0]*2 >= MAX_FINGER_APERTURE){
      std::cout << "RAVE: selected top grasp for " << obj_name << std::endl;
      if(object_size[1]*2 < MAX_FINGER_APERTURE){
        grasp.pose.rot = FRONT_TOP_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x;
        grasp.pose.trans.y = object_pose.trans.y;
        grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
      }
    }
  }


  return grasp;
}

Grasp FRASIEROpenRAVE::generateGraspPose(int table_pose){
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();
  Grasp grasp;
  while(true){

    std::vector <OpenRAVE::KinBodyPtr> bodies;
    OpenRAVE::KinBodyPtr grasp_body;
    env_->GetBodies(bodies);
    double closest_distance = std::numeric_limits<double>::max();
    for (const auto body : bodies) {
      std::string body_name = body->GetName();

      if (body_name.substr(0, 9) == "table_obj"){

        OpenRAVE::Transform obj_pose = hsr_pose.inverse() * body->GetTransform();

        double distance = std::sqrt(std::pow(obj_pose.trans.x, 2) + std::pow(obj_pose.trans.y, 2));
        if (distance < closest_distance){
          grasp_body = body;
          closest_distance = distance;

        }
      }

    }

    grasp.obj_name = grasp_body->GetName();

    OpenRAVE::Transform object_pose = grasp_body->GetTransform();
    OpenRAVE::Vector object_size = grasp_body->GetLink("base")->GetGeometry(0)->GetBoxExtents();

//  if(object_size[0]*2 < MAX_FINGER_APERTURE){
//    std::cout << "RAVE: selected side grasp for " << grasp_body->GetName() << std::endl;
//    grasp.pose.rot = LEFT_EEF_ROT;
//    grasp.pose.trans.x = object_pose.trans.x;
//    grasp.pose.trans.y = object_pose.trans.y - 0.02;
//    grasp.pose.trans.z = object_pose.trans.z;
//  }
//  else if(object_size[0]*2 >= MAX_FINGER_APERTURE){
//    if(object_size[1]*2 < MAX_FINGER_APERTURE){
//      std::cout << "RAVE: selected top grasp for " << grasp_body->GetName() << std::endl;
//      grasp.pose.rot = FRONT_TOP_EEF_ROT;
//      grasp.pose.trans.x = object_pose.trans.x;
//      grasp.pose.trans.y = object_pose.trans.y;
//      grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
//    }
//  }

    if(table_pose == FRONT_TABLE){
      std::cout << "RAVE: table is front!" << std::endl;
      if(object_size[1]*2 < MAX_FINGER_APERTURE){
        std::cout << "RAVE: selected side grasp for " << grasp.obj_name << std::endl;
        grasp.pose.rot = FRONT_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x - 0.02;
        grasp.pose.trans.y = object_pose.trans.y;
        grasp.pose.trans.z = object_pose.trans.z;
        grasp.graspable = true;
      }
      else if(object_size[1]*2 >= MAX_FINGER_APERTURE){
        if(object_size[0]*2 < MAX_FINGER_APERTURE){
          std::cout << "RAVE: selected top grasp for " << grasp.obj_name << std::endl;
          grasp.pose.rot = FRONT_TOP_EEF_ROT;
          grasp.pose.trans.x = object_pose.trans.x;
          grasp.pose.trans.y = object_pose.trans.y;
          grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
          grasp.graspable = true;
        }
      } else{
        grasp.graspable = false;
      }
    }
    else if(table_pose == LEFT_TABLE){
      std::cout << "RAVE: table is left!" << std::endl;
      if(object_size[0]*2 < MAX_FINGER_APERTURE){
        std::cout << "RAVE: selected side grasp for " << grasp.obj_name << std::endl;
        grasp.pose.rot = LEFT_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x;
        grasp.pose.trans.y = object_pose.trans.y - 0.02;
        grasp.pose.trans.z = object_pose.trans.z;
        grasp.graspable = true;
      }
      else if(object_size[0]*2 >= MAX_FINGER_APERTURE){
        if(object_size[1]*2 < MAX_FINGER_APERTURE){
          std::cout << "RAVE: selected top grasp for " << grasp.obj_name << std::endl;
          grasp.pose.rot = FRONT_TOP_EEF_ROT;
          grasp.pose.trans.x = object_pose.trans.x;
          grasp.pose.trans.y = object_pose.trans.y;
          grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
          grasp.graspable = true;
        }
      } else{
        grasp.graspable = false;
      }
    }
    else if(table_pose == RIGHT_TABLE){
      std::cout << "RAVE: table is right!"  << std::endl;
      if(object_size[0]*2 < MAX_FINGER_APERTURE){
        std::cout << "RAVE: selected side grasp for " << grasp.obj_name << std::endl;
        grasp.pose.rot = RIGHT_EEF_ROT;
        grasp.pose.trans.x = object_pose.trans.x;
        grasp.pose.trans.y = object_pose.trans.y + 0.02;
        grasp.pose.trans.z = object_pose.trans.z;
        grasp.graspable = true;
      }
      else if(object_size[0]*2 >= MAX_FINGER_APERTURE){

        if(object_size[1]*2 < MAX_FINGER_APERTURE){
          std::cout << "RAVE: selected top grasp for " << grasp.obj_name << std::endl;
          grasp.pose.rot = FRONT_TOP_EEF_ROT;
          grasp.pose.trans.x = object_pose.trans.x;
          grasp.pose.trans.y = object_pose.trans.y;
          grasp.pose.trans.z = object_pose.trans.z + object_size[2] + 0.01;
          grasp.graspable = true;
        }
      } else{
        grasp.graspable = false;
      }
    }
    if(!grasp.graspable){
      OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
      std:: string not_graspable_obj_name = "not_graspable_" + grasp.obj_name;
      grasp_body->SetName(not_graspable_obj_name);
      std::cout << "RAVE: object not graspable." << std::endl;
      continue;
    }
    grasp.graspable = true;
    break;

  }
  return grasp;

}

OpenRAVE::Transform FRASIEROpenRAVE::generatePlacePose(std::string &obj_name) {
  OpenRAVE::Transform place_pose;
  place_pose.rot = FRONT_EEF_ROT;

  OpenRAVE::KinBodyPtr object = env_->GetKinBody(obj_name);
  OpenRAVE::Transform object_pose = object->GetTransform();
  std::string shelf_name = "rack_1";
  OpenRAVE::KinBodyPtr shelf = env_->GetKinBody(shelf_name);
  OpenRAVE::Transform shelf_pose = shelf->GetTransform();

  if(object_pose.trans.y < shelf_pose.trans.y){
    place_pose.trans = object_pose.trans;
    place_pose.trans.y = place_pose.trans.y + 0.15;
    place_pose.trans.x = place_pose.trans.x + 0.03;
  }else{
    place_pose.trans = object_pose.trans;
    place_pose.trans.y = place_pose.trans.y - 0.15;
    place_pose.trans.x = place_pose.trans.x + 0.03;
  }

  return place_pose;
}

std::vector<OpenRAVE::Transform> FRASIEROpenRAVE::generatePlacePoses(){
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

  std::vector<OpenRAVE::Transform> place_poses, mid_place_poses, left_place_poses, right_place_poses;

  std::vector <OpenRAVE::KinBodyPtr> bodies;
  env_->GetBodies(bodies);

  for (const auto body : bodies) {
    std::string body_name = body->GetName();

    if (body_name.substr(0, 4) == "rack"){
      std::cout << "RAVE: creating place poses for " << body_name << std::endl;

//      OpenRAVE::Transform place_pose = hsr_pose.inverse() * bodies[i]->GetTransform();
      OpenRAVE::Transform rack_pose = body->GetTransform();
      OpenRAVE::Transform mid_place_pose, left_place_pose, right_place_pose;

      mid_place_pose.rot = FRONT_EEF_ROT;
      mid_place_pose.trans.x = rack_pose.trans.x;
      mid_place_pose.trans.y = rack_pose.trans.y;
      mid_place_pose.trans.z = rack_pose.trans.z + 0.1;

      mid_place_poses.push_back(mid_place_pose);

      left_place_pose.rot = FRONT_EEF_ROT;
      left_place_pose.trans.x = rack_pose.trans.x;
      left_place_pose.trans.y = rack_pose.trans.y + 0.15;
      left_place_pose.trans.z = rack_pose.trans.z + 0.1;

      left_place_poses.push_back(left_place_pose);

      right_place_pose.rot = FRONT_EEF_ROT;
      right_place_pose.trans.x = rack_pose.trans.x;
      right_place_pose.trans.y = rack_pose.trans.y - 0.15;
      right_place_pose.trans.z = rack_pose.trans.z + 0.1;

      right_place_poses.push_back(right_place_pose);

    }

  }


  for (auto pose : mid_place_poses) {
    place_poses.push_back(pose);
  }

  for (auto pose : left_place_poses) {
    place_poses.push_back(pose);
  }

  for (auto pose : right_place_poses) {
    place_poses.push_back(pose);
  }


  return place_poses;
}
