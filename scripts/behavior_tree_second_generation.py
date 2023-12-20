#!/usr/bin/env python3
import argparse
import os

import openai

from utils import post_processing, subtree_assembly


def LLM_generation(**kwargs):
    openai.api_key = os.environ["OPENAI_API_KEY"]

    for ind in kwargs['wrong_raw_indices']:
        env_subtree_description_string = open(os.path.join(kwargs['prompt_description_dir'], kwargs['generation_name'], "env_subtree.txt"), "rt").read()

        error_description_path = os.path.join(kwargs['prompt_description_dir'], "error_examples.txt")
        print(f"Reading error_examples.txt: {error_description_path}")
        error_description_string = open(error_description_path, "rt").read()

        task_description = open(os.path.join(kwargs['prompt_description_dir'], kwargs['generation_name'], "task_description.txt"), "rt").read()

        wrong_raw_description = "\n\n" + "-"*100 + "\n\nWrong format example (should be modified) of behavior tree architecture:\n\n"

        wrong_raw_path = os.path.join(kwargs['src_generate_dir'], "raw", f"raw{ind}.txt")
        print(f"Reading wrong raw.txt: {wrong_raw_path}")
        wrong_raw_string = open(wrong_raw_path, "rt").read()

        right_raw_description = "\n\n" + "-"*100 + "\n\nRight format example of behavior tree architecture:\n\n"

        right_raw_path = os.path.join(kwargs['prompt_description_dir'], "right_format_bt.txt")
        print(f"Reading right raw.txt: {right_raw_path}")
        right_raw_string = open(right_raw_path, "rt").read()

        ask = "\n\n" + "-"*100 + "\n\nPlease correct the wrong format example (that should be modified) in correct behavior tree architecture. You can correct the errors with the error examples and reference the right format of example. "
        
        second_prompt = env_subtree_description_string + error_description_string + task_description + wrong_raw_description + wrong_raw_string + right_raw_description + right_raw_string + ask 
        
        print(f"Generating prompt_raw_{ind}.txt")
        with open(os.path.join(kwargs['dst_generate_dir'], f"prompt_raw_{ind}.txt"), "w+") as file:
            file.write(second_prompt)

        response = openai.Completion.create(
            model=kwargs['model_name'],
            prompt=second_prompt,
            temperature=0,
            max_tokens=500,
            top_p=1,
            presence_penalty=0,
            frequency_penalty=0.2
        )

        # raw_string = str(response['choices'][0]['text'])

        # generate_file_dir = os.path.join(kwargs['dst_generate_dir'], "generated_files")
        # os.makedirs(generate_file_dir, exist_ok=True)
        # raw_path = os.path.join(generate_file_dir, f"raw_{ind}.txt")

        # with open(raw_path, "wt+") as raw_text_file:
        #     raw_text_file.write(raw_string)

        response_dir = os.path.join(kwargs['dst_generate_dir'], "response")
        if not os.path.exists(response_dir):
            os.makedirs(response_dir)

        response_path = os.path.join(kwargs['dst_generate_dir'], "response", f"response{ind}.txt")
        response_str = str(response)
        with open(response_path, "w+") as file:
            file.write(response_str)

        raw_dir = os.path.join(kwargs['dst_generate_dir'], "raw")
        if not os.path.exists(raw_dir):
            os.makedirs(raw_dir)

        raw_path = os.path.join(kwargs['dst_generate_dir'], "raw", f"raw{ind}.txt")
        raw_string = str(response["choices"][0]["text"])
        with open(raw_path, "w+") as file:
            file.write(raw_string)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Behavior Tree Generation Script")
    parser.add_argument("--generation_name", type=str, default="inspection", help="Name of the generation")
    parser.add_argument("--model_name", type=str, default="text-davinci-003", help="Model name for generation")
    parser.add_argument("--wrong_raw_indices_string", type=str, default="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29", help="Indices for wrong raw data")

    args = parser.parse_args()

    # Determine the base directory of the project
    script_dir = os.path.dirname(os.path.realpath(__file__))
    base_dir = os.path.dirname(script_dir)

    src_generate_dir = os.path.join(base_dir, "config", "llm-bt-gpt", "gpt3_temperature_1", "M+L", args.generation_name)
    dst_generate_dir = os.path.join(base_dir, "config", "llm-bt-gpt", "gpt3_temperature_1_second_prompt", "M+L", args.generation_name)
    if not os.path.exists(dst_generate_dir):
        os.makedirs(dst_generate_dir)

    sub_tree_dir = os.path.join(base_dir, "config", "subtree")
    wrong_raw_indices = args.wrong_raw_indices_string.split(",")
    prompt_description_dir = os.path.join(base_dir, "config", "second_prompt")

    # Calling LLM_generation with keyword arguments
    LLM_generation(
        src_generate_dir=src_generate_dir,
        dst_generate_dir=dst_generate_dir,
        wrong_raw_indices=wrong_raw_indices,
        model_name=args.model_name,
        prompt_description_dir=prompt_description_dir,
        generation_name=args.generation_name,
        sub_tree_dir=sub_tree_dir
    )
