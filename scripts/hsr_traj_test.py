#!/usr/bin/env python

import openravepy
import trajoptpy
import json

if __name__ == "__main__":
    env = openravepy.Environment()
    # env.Load('../worlds/hsr_arm_base.xml')
    env.Load('../worlds/pr2_table.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobot('hsrb')


    request = {
      "basic_info" : {
        "n_steps" : 9,
        "manip" : "rightarm",
        "start_fixed" : True
      },
      "costs" : [
      {
        "name" : "disc_coll",
        "type" : "collision",
        "params" :
        {
          "coeffs" : [ 50 ],
          "continuous" : True,
          "dist_pen" : [ 0.040 ],
          "first_step" : 0,
          "last_step" : 8
        }
        }
      ],
      "constraints" : [
      {
        "type" : "pose",
        "params" : {
          "xyz" : [2.0, 0.5, 0.8],
          "wxyz": [0.966, 0.0, 0.0, 0.259],
          "link": "r_gripper_tool_frame",
          "timestep" : 7
                    }

      }
      ],

      "init_info" : {
          "type" : "stationary"
      }
    }

    s = json.dumps(request) # convert dictionary into json-formatted string
    # prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem

    import IPython
    IPython.embed()
