#!/usr/bin/env python

import openravepy
# import trajoptpy
# import json

if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('../worlds/hsr_empty_world.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobot('hsrb')
    # manip = robot.GetActiveManipulator()
    import IPython
    IPython.embed()
