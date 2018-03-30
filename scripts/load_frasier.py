#!/usr/bin/env python

import openravepy

if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('../worlds/hsr_whole_body.xml')
    # env.Load('../worlds/pr2_empty_world.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobot('hsrb')
    import IPython
    IPython.embed()
