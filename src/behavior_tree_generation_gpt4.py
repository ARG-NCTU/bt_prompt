import os
import argparse
import openai
from utils import post_processing, subtree_assembly

def LLM_generation(args):
    openai.api_key = os.getenv("OPENAI_API_KEY")
    if openai.api_key is None:
        print("OPENAI_API_KEY is not set")
        return
    print("OpenAI API Key: {}".format(openai.api_key))

    # Create necessary directories
    os.makedirs(os.path.join(args.generate_dir, "response"), exist_ok=True)
    os.makedirs(os.path.join(args.generate_dir, "raw"), exist_ok=True)
    os.makedirs(os.path.join(args.generate_dir, "tree"), exist_ok=True)

    print("Reading prompt.txt")
    prompt_path = os.path.join(args.prompt_dir, "prompt.txt")
    if not os.path.exists(prompt_path):
        print("prompt.txt not found in specified directory")
        return
    with open(prompt_path, "r") as file:
        prompt = file.read()

    print("Generating response")
    from openai import OpenAI
    client = OpenAI()
    response = client.chat.completions.create(
        model=args.model_name,
        messages=[{"role": "user", "content": prompt}],
        temperature=args.temp,
        max_tokens=1000,
        top_p=1,
        presence_penalty=0,
        frequency_penalty=0.2,
    )

    # Write outputs to respective files
    response_str = str(response)
    response_path = os.path.join(args.generate_dir, "response", f"response{args.count}.txt")
    print(f"Writing response.txt: {response_path}")
    with open(response_path, "w") as file:
        file.write(response_str)

    # raw_string = response["choices"][0]["message"]["content"]
    raw_string = response.choices[0].message.content
    raw_path = os.path.join(args.generate_dir, "raw", f"raw{args.count}.txt")
    print(f"Writing raw.txt: {raw_path}")
    with open(raw_path, "w") as file:
        file.write(raw_string)

    try:
        tree_path = os.path.join(args.generate_dir, "tree", f"tree{args.count}.txt")
        tree = post_processing(raw_string)
        tree = subtree_assembly(tree, args.sub_tree_dir)
        with open(tree_path, "w") as file:
            file.write(tree)
    except Exception as e:
        print(f"Error processing tree: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate LLM responses and process trees.")
    parser.add_argument("--prompt_dir", type=str, default="../config/prompt", help="Directory to store generated outputs.")
    parser.add_argument("--generate_dir", type=str, default="../results", help="Directory to store generated outputs.")
    parser.add_argument("--model_name", type=str, default="gpt-4-0613", help="Model name for OpenAI API.")
    parser.add_argument("--sub_tree_dir", type=str, default="../config/subtree/", help="Directory for subtree configuration.")
    parser.add_argument("--count", type=int, default=1, help="Number of times to generate outputs.")
    parser.add_argument("--temp", type=float, default=1.0, help="Temperature for response generation.")
    args = parser.parse_args()

    for i in range(args.count):
        args.count = i  # Update the count attribute for each iteration
        LLM_generation(args)
