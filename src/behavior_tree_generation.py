#!/usr/bin/env python3
import os

import openai
import rospkg
import rospy

from bt_generator import BTGenerator
from utils import post_processing, subtree_assembly


def main(path, sub_tree_dir_path, count,
         model="gpt-3.5-turbo-instruct", temperature=0, **kwargs):
    bt_gen = BTGenerator(
        path=path,
        sub_tree_dir_path=sub_tree_dir_path,
        model=model,
        temperature=temperature,
        **kwargs)
    bt_gen.read_prompt()

    for i in range(count):
        response = bt_gen.generate()
        bt_gen.write_response(response, count=i)
        bt_gen.write_raw(response, count=i)
        bt_gen.write_tree(bt_gen.generate_tree(response), count=i)


if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_node")
    rospack = rospkg.RosPack()

    generate_dir = rospy.get_param("~generate_dir",
                                   os.path.join(rospack.get_path("behavior_tree_generation"),
                                                "config",
                                                "exp",
                                                "L",
                                                "test"))
    model_name = rospy.get_param("~model_name", "gpt-3.5-turbo-instruct")
    temperature = rospy.get_param("~temperature", 0)
    count = rospy.get_param("~count", 30)
    sub_tree_dir = rospack.get_path(
        "behavior_tree_generation") + "/config/subtree/"

    main(
        generate_dir,
        sub_tree_dir,
        count,
        model=model_name,
        temperature=temperature)
