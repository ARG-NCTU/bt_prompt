#!/usr/bin/env python3
import os
import rospkg
import rospy

from openai import OpenAI, OpenAIError

client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"),)

def LLM_generation(args, count=None):
    # if client is None:
    #     raise Exception("OpenAI client is not initialized")

    os.makedirs(os.path.join(args.generate_dir, "response"), exist_ok=True)
    os.makedirs(os.path.join(args.generate_dir, "raw", args.model_name, args.gen_file), exist_ok=True)

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
        response = client.chat.completions.create(
            model=args.model_name,
            messages=[
                {"role": "system", "content": "You will be provided with a piece of prompt file, and your task is to generate a json format file that follows the rule described in the prompt  and satisfies the task description."},
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

        raw_path = os.path.join(args.generate_dir, "raw", args.model_name, args.gen_file, f"raw{count}.json")
        rospy.loginfo(f"Writing raw.json: {raw_path}")
        raw_string = response.choices[0].message.content

    except OpenAIError as e:
        rospy.logerr(f"OpenAI API error: {e}")
        rospy.loginfo("Fallback to ChatCompletion API")

        response = client.chat.completions.create(
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

        raw_path = os.path.join(args.generate_dir, "raw", args.model_name, args.gen_file, f"raw{count}.txt")
        rospy.loginfo(f"Writing raw.json: {raw_path}")
        raw_string = response.choices[0].message.content

    with open(raw_path, "w+") as file:
        file.write(raw_string)

    rospy.loginfo("Generation results: \n" + raw_string + "\n")

class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)

if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_node")
    rospack = rospkg.RosPack()

    mission_name = rospy.get_param("~mission_name", "test")
    gen_file = rospy.get_param("~gen_file", "moos")
    model_name = rospy.get_param("~model_name", "")
    count = rospy.get_param("~count", 30)
    generate_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "moos", mission_name)

    args = {
        "generate_dir": generate_dir,
        "gen_file": gen_file,
        "model_name": model_name,
    }
    args = Args(args)

    for i in range(count):
        LLM_generation(args, count=i)
        if rospy.is_shutdown():
            break
