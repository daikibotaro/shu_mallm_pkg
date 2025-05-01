#!/usr/bin/env python3

import rospy
import subprocess


def main():
    rospy.init_node('run_chainlit')
    rospy.loginfo("run_chainlit node started!")
    try:
        # Chainlitを起動するコマンドを実行
        result = subprocess.run(["python3", "/home/hiratalab/catkin_ws/src/shu_task_planning/my_package/run.py"])
        rospy.loginfo("Subprocess output: %s", result.stdout)
    except subprocess.CalledProcessError as e:
        rospy.logerr("Subprocess failed with error: %s", e)

    rospy.spin()


if __name__ == "__main__":
    main()
