#!/usr/bin/env python3
import os
import time

import openai
import rospkg
import rospy
from utils import post_processing, subtree_assembly


def LLM_generation(args):
    
    for ind in args.wrong_raw_indices:
        
        env_subtree_description_string = open(os.path.join(args.prompt_description_dir, args.generation_name, "env_subtree.txt"), "rt").read()
        
        error_description_path = os.path.join(args.prompt_description_dir, "error_examples.txt")
        rospy.loginfo(f"Reading error_examples.txt: {error_description_path}")
        error_description_string = open(error_description_path, "rt").read()

        task_description = open(os.path.join(args.prompt_description_dir, args.generation_name, "task_description.txt"), "rt").read()

        wrong_raw_description = "\n\n" + "-"*100 + "\n\nWrong format example (should be modified) of behavior tree architecture:\n\n"

        wrong_raw_path = os.path.join(args.src_generate_dir, "raw", f"{ind}.txt")
        rospy.loginfo(f"Reading wrong raw.txt: {wrong_raw_path}")
        wrong_raw_string = open(wrong_raw_path, "rt").read()

        right_raw_description = "\n\n" + "-"*100 + "\n\nRight format example of behavior tree architecture:\n\n"

        right_raw_path = os.path.join(args.prompt_description_dir, "right_format_bt.txt")
        rospy.loginfo(f"Reading right raw.txt: {right_raw_path}")
        right_raw_string = open(right_raw_path, "rt").read()
        
        ask = "\n\n" + "-"*100 + "\n\nPlease correct the wrong format example (that should be modified) in correct behavior tree architecture. You can correct the errors with the error examples and reference the right format of example. "
        
        prompt = env_subtree_description_string \
                + error_description_string \
                + task_description \
                + wrong_raw_description \
                + wrong_raw_string \
                + right_raw_description \
                + right_raw_string \
                    + ask + "\nWrite in text file format for me to copy."
        
        rospy.loginfo(f"Generating prompt_raw_{ind}.txt")
        with open(os.path.join(args.dst_generate_dir, f"prompt_raw_{ind}.txt"), "w+") as file:
            file.write(prompt)

class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)


if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_node")
    rospack = rospkg.RosPack()

    generation_name = rospy.get_param("~generation_name", "test")
    model_name = rospy.get_param("~model_name", "")
    src_generate_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "llm-bt-gpt", "gpt3.5_online",generation_name)
    dst_generate_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "llm-bt-gpt", "gpt3.5_online_2", "M+L",generation_name)
    if not os.path.exists(dst_generate_dir):
        os.makedirs(dst_generate_dir)

    sub_tree_dir = rospack.get_path("behavior_tree_generation") + "/config/subtree/"

    wrong_raw_indices_string = rospy.get_param("~wrong_raw_indices_string", None)
    wrong_raw_indices = wrong_raw_indices_string.split(",")
    
    prompt_description_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "second_prompt")

    args = {
        "src_generate_dir": src_generate_dir,
        "dst_generate_dir": dst_generate_dir,
        "wrong_raw_indices": wrong_raw_indices,
        "model_name": model_name,
        "prompt_description_dir": prompt_description_dir,
        "generation_name": generation_name,
        "sub_tree_dir": sub_tree_dir,
    }
    args = Args(args)

    LLM_generation(args)
