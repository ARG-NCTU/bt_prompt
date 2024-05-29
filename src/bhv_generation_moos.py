#!/usr/bin/env python3
import os
import time

import openai
import rospkg
import rospy
from utils import post_processing, subtree_assembly


def LLM_generation(args, count=None):
    openai.api_key = os.getenv("OPENAI_API_KEY")
    if openai.api_key is None:
        rospy.logerr("OPENAI_API_KEY is not set")
        return
    rospy.loginfo("OpenAI API Key: {}".format(openai.api_key))

    os.makedirs(os.path.join(args.generate_dir, "response"), exist_ok=True)
    os.makedirs(os.path.join(args.generate_dir, "raw", args.model_name), exist_ok=True)
    os.makedirs(os.path.join(args.generate_dir, "tree"), exist_ok=True)

    prompt_file_name = "prompt.txt" 
    rospy.loginfo(prompt_file_name)
    try:
        prompt = open(os.path.join(args.generate_dir, prompt_file_name), "rt").read()
    except FileNotFoundError:
        # Write a default prompt into the file
        prompt = "Hi, GPT!"
        with open(os.path.join(args.generate_dir, prompt_file_name), "wt") as file:
            file.write(prompt)

    rospy.loginfo("Generating response")
    try:
        response = openai.Completion.create(
            model=args.model_name,
            prompt=prompt,
            temperature=0,
            max_tokens=4096,
            top_p=1,
            presence_penalty=0,
            frequency_penalty=0.2,
        )
        
        response_path = os.path.join(args.generate_dir, "response", f"response{count}.txt")
        rospy.loginfo(f"Writing response.txt: {response_path}")
        response_str = str(response)
        with open(response_path, "w+") as file:
            file.write(response_str)

        raw_path = os.path.join(args.generate_dir, "raw", args.model_name, f"raw{count}.txt")
        rospy.loginfo(f"Writing raw.txt: {raw_path}")
        raw_string = str(response["choices"][0]["text"])

    except openai.error.OpenAIError as e:
        rospy.logerr(f"OpenAI API error: {e}")
        rospy.loginfo("Fallback to ChatCompletion API")
        
        response = openai.ChatCompletion.create(
            model=args.model_name,
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt},
            ],
            temperature=0,
            max_tokens=4096,
            top_p=1,
            presence_penalty=0,
            frequency_penalty=0.2,
        )

        response_path = os.path.join(args.generate_dir, "response", f"response{count}.txt")
        rospy.loginfo(f"Writing response.txt: {response_path}")
        response_str = str(response)
        with open(response_path, "w+") as file:
            file.write(response_str)

        raw_path = os.path.join(args.generate_dir, "raw", args.model_name, f"raw{count}.txt")
        rospy.loginfo(f"Writing raw.txt: {raw_path}")
        raw_string = response["choices"][0]["message"]["content"]

    with open(raw_path, "w+") as file:
        file.write(raw_string)

    rospy.loginfo("Generation results: \n" + raw_string + "\n")

    tree_path = os.path.join(args.generate_dir, "tree", f"test{count}.tree")
    try:
        tree = post_processing(raw_string)
        tree = subtree_assembly(tree, args.sub_tree_dir)
        with open(tree_path, "w+") as file:
            file.write(tree)
    except Exception as e:
        rospy.logerr(f"Error in post_processing: {e}")

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

    for i in range(count):
        LLM_generation(args, count=i)
        if rospy.is_shutdown():
            break
