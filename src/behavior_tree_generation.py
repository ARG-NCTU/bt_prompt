#!/usr/bin/env python3
import os

import rospkg
import rospy

from bt_prompt.bt_generator import BTGenerator


def main(path, sub_tree_dir_path, count,
         model="gpt-3.5-turbo-instruct", temperature=0, **kwargs):
    """
    Generate behavior trees using the specified model and parameters.

    Args:
        path (str): The path to the behavior tree file.
        sub_tree_dir_path (str): The path to the directory where sub-trees are stored.
        count (int): The number of behavior trees to generate.
        model (str, optional): The model to use for generation. Defaults to "gpt-3.5-turbo-instruct".
        temperature (int, optional): The temperature parameter for generation. Defaults to 0.
        **kwargs: Additional keyword arguments for the BTGenerator.

    Returns:
        None
    """
    bt_gen = BTGenerator(
        path=path,
        sub_tree_dir_path=sub_tree_dir_path,
        model=model,
        temperature=temperature,
        **kwargs)
    bt_gen.read_prompt()

    for i in range(count):
        rospy.loginfo(f"Generating BT {i}")
        response = bt_gen.generate()
        bt_gen.write_response(response, count=i)
        bt_gen.write_raw(response, count=i)
        bt_gen.write_tree(bt_gen.generate_tree(response), count=i)
        if rospy.is_shutdown():
            break


if __name__ == "__main__":
    rospy.init_node("bt_prompt_node")
    rospack = rospkg.RosPack()

    generate_dir = rospy.get_param("~generate_dir",
                                   os.path.join(rospack.get_path("bt_prompt"),
                                                "config",
                                                "exp",
                                                "L",
                                                "test"))
    model_name = rospy.get_param("~model_name", "gpt-3.5-turbo-instruct")
    temperature = rospy.get_param("~temperature", 0)
    count = rospy.get_param("~count", 30)
    sub_tree_dir = rospack.get_path(
        "bt_prompt") + "/config/subtree/"

    main(
        generate_dir,
        sub_tree_dir,
        count,
        model=model_name,
        temperature=temperature)
