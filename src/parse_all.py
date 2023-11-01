#!/usr/bin/env python3
import os
import traceback

import rospkg
import rospy
from utils import post_processing, subtree_assembly


def LLM_generation(args, count=None):
    os.makedirs(os.path.join(args.generate_dir, "raw"), exist_ok=True)

    rospy.loginfo(f"Reading raw.txt: {count}")
    raw_txt = open(os.path.join(args.generate_dir, "raw", f"raw{count}.txt")).read()

    tree_path = os.path.join(args.generate_dir, "tree", f"test{count}.tree")
    try:
        tree = post_processing(raw_txt)
        print(tree)
        tree = subtree_assembly(tree, args.sub_tree_dir)
        with open(tree_path, "w+") as file:
            file.write(tree)
    except Exception as e:
        rospy.logerr(f"Error[{count}]: {e}")
        traceback.print_exc()
        return

    rospy.loginfo("Start sleeping")
    for i in range(args.generate_time_interval * args.sleep_seperate):
        rospy.sleep(1.0 / args.sleep_seperate)
        print(
            f"[INFO] [{rospy.get_time():.6f}]: Sleeping progress: {i / args.generate_time_interval / args.sleep_seperate * 100.0: .2f}%",
            end="\r",
        )
        if rospy.is_shutdown():
            return


class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)


if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_node")
    rospack = rospkg.RosPack()

    generation_name = rospy.get_param("~generation_name", "test")
    model_name = rospy.get_param("~model_name", "")
    count = rospy.get_param("~count", 30)
    generate_time_interval = rospy.get_param("~generate_time_interval", 20)
    generate_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "exp", "L", generation_name)
    sleep_seperate = rospy.get_param("~sleep_seperate", 100)

    sub_tree_dir = rospack.get_path("behavior_tree_generation") + "/config/subtree/"

    args = {
        "generate_dir": generate_dir,
        "model_name": model_name,
        "generate_time_interval": generate_time_interval,
        "sleep_seperate": sleep_seperate,
        "sub_tree_dir": sub_tree_dir,
    }
    args = Args(args)
    # LLM_generation(args, count)
    # exit()

    for i in range(count):
        LLM_generation(args, count=i)
        if rospy.is_shutdown():
            break
