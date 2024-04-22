# This code use python3 in ubuntu 20.04, no ROS needed
import argparse
import os
from bt_generator import BTGenerator


def main(args):
    """
    Generates a response using OpenAI's GPT-4 model based on the provided prompt.

    Args:
        args (object): An object containing the necessary arguments for generating the response.
            - prompt_dir (str): Directory to store generated outputs.
            - generate_dir (str): Directory to store generated outputs.
            - model_name (str): Model name for OpenAI API.
            - sub_tree_dir (str): Directory for subtree configuration.
            - count (int): Number of times to generate outputs.
            - temp (float): Temperature for response generation.

    Returns:
        None
    """
    bt_gen = BTGenerator(
        path=args.generate_dir,
        sub_tree_dir_path=args.sub_tree_dir,
        model=args.model_name,
        temperature=args.temp,
        )
    bt_gen.read_prompt(prompt_path=os.path.join(args.prompt_dir, args.prompt_file))

    for i in range(args.count):
        print(f"Generating BT {i}")
        response = bt_gen.chat_generate()
        bt_gen.write_response(response, count=i)
        bt_gen.write_raw(response, count=i)
        bt_gen.write_tree(bt_gen.generate_tree(response), count=i)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate LLM responses and process trees.")
    parser.add_argument(
        "--prompt_dir",
        type=str,
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/prompt"),
        help="Directory to store generated outputs.")
    parser.add_argument(
        "--generate_dir",
        type=str,
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "../results"),
        help="Directory to store generated outputs.")
    parser.add_argument(
        "--model_name",
        type=str,
        default="gpt-4-0613",
        help="Model name for OpenAI API.")
    parser.add_argument(
        "--sub_tree_dir",
        type=str,
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/subtree/"),
        help="Directory for subtree configuration.")
    parser.add_argument(
        "--prompt_file",
        type=str,
        default="prompt.txt",
        help="File name containing the prompt."
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of times to generate outputs.")
    parser.add_argument(
        "--temp",
        type=float,
        default=1.0,
        help="Temperature for response generation.")
    args = parser.parse_args()

    main(args)
