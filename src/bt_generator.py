import os

from openai_wrapper import OpenAIWrapper
from utils import post_processing, subtree_assembly


class BTGenerator:
    def __init__(self, path, sub_tree_dir_path, model="gpt-3.5-turbo-instruct",
                 temperature=0,
                 max_tokens=500,
                 top_p=1,
                 presence_penalty=0,
                 frequency_penalty=0.2, **kwargs):
        """
        Initializes a BTGenerator object.

        Args:
            path (str): The path to the directory where the files will be stored.
            sub_tree_dir_path (str): The path to the sub-tree directory.
            model (str, optional): The model to use for generating responses. Defaults to "gpt-3.5-turbo-instruct".
            temperature (float, optional): Controls the randomness of the generated responses. Defaults to 0.
            max_tokens (int, optional): The maximum number of tokens in the generated response. Defaults to 500.
            top_p (float, optional): Controls the diversity of the generated responses. Defaults to 1.
            presence_penalty (float, optional): Controls the model's preference for including or avoiding certain phrases. Defaults to 0.
            frequency_penalty (float, optional): Controls the model's preference for using new or repeated phrases. Defaults to 0.2.
            **kwargs: Additional keyword arguments to be passed to the OpenAIWrapper.

        """
        self.path = os.path.expanduser(path)
        self.sub_tree_dir_path = os.path.expanduser(sub_tree_dir_path)
        self.model = model
        self.wrapper = OpenAIWrapper(model=model,
                                     temperature=temperature,
                                     max_tokens=max_tokens,
                                     top_p=top_p,
                                     presence_penalty=presence_penalty,
                                     frequency_penalty=frequency_penalty,
                                     **kwargs)

        self.prompt = None
        self.response = None
        self.tree = None

        # File tree is like:
        # path/
        # ├── prompt.txt
        # ├── response/
        # ├── raw/
        # └── tree/
        os.makedirs(os.path.join(self.path, "response"), exist_ok=True)
        os.makedirs(os.path.join(self.path, "raw"), exist_ok=True)
        os.makedirs(os.path.join(self.path, "tree"), exist_ok=True)

    def read_prompt(self, prompt_path=None):
        """
        Reads the prompt from a file.

        Args:
            prompt_path (str, optional): The path to the prompt file. If not provided, the default path is used.

        Returns:
            None

        Raises:
            FileNotFoundError: If the prompt file is not found.

        """
        if prompt_path is None:
            prompt_path = os.path.join(self.path, "prompt.txt")
        try:
            self.prompt = open(prompt_path, "rt").read()
        except FileNotFoundError:
            # Write a default prompt into the file
            self.prompt = "Hi, GPT!"
            with open(prompt_path, "wt") as file:
                file.write(self.prompt)
        os.chmod(prompt_path, 0o666)

    def generate(self):
        """
        Generates a response using the wrapper's completion_create method.

        Returns:
            str: The generated response.
        """
        self.response = self.wrapper.completion_create(self.prompt)
        return self.response

    def write_response(self, response=None, count=None):
        """
        Writes the given response to a file.

        Args:
            response (str, optional): The response to write. If not provided, uses the default response.
            count (int, optional): The count used to generate the response file name. If not provided, uses a default count.

        Returns:
            None
        """
        if response is None:
            response = self.response

        if count is None:
            response_path = os.path.join(self.path, "response", "response.txt")
        else:
            response_path = os.path.join(
                self.path, "response", f"response{count}.txt")

        with open(response_path, "w+") as file:
            file.write(self.wrapper.dump_response_from_response(response))
        os.chmod(response_path, 0o666)

    def write_raw(self, response=None, count=None):
        """
        Writes the raw text from the response to a file.

        Args:
            response (str, optional): The response to write to the file. If not provided, uses the stored response.
            count (int, optional): The count used to generate the filename. If not provided, uses a default filename.

        Returns:
            None
        """
        if response is None:
            response = self.response

        if count is None:
            raw_path = os.path.join(self.path, "raw", "raw.txt")
        else:
            raw_path = os.path.join(self.path, "raw", f"raw{count}.txt")

        with open(raw_path, "w+") as file:
            file.write(self.wrapper.get_text_from_response(response))
        os.chmod(raw_path, 0o666)

    def generate_tree(self, response=None):
        """
        Generates a behavior tree based on the given response.

        Args:
            response (optional): The response to generate the behavior tree from. If not provided, uses the stored response.

        Returns:
            None

        Raises:
            Exception: If an error occurs during tree generation.

        """
        if response is None:
            response = self.response

        raw_string = self.wrapper.get_text_from_response(response)

        try:
            self.tree = post_processing(raw_string)
            self.tree = subtree_assembly(self.tree, self.sub_tree_dir_path)
        except Exception as e:
            print(e)
            self.tree = None

    def write_tree(self, tree=None, count=None):
        """
        Writes the given tree to a file.

        Args:
            tree (str): The tree to be written. If None, uses the tree stored in the object.
            count (int): The count used to generate the file name. If None, uses a default file name.

        Returns:
            None
        """
        if tree is None:
            tree = self.tree

        if count is None:
            tree_path = os.path.join(self.path, "tree", "tree.tree")
        else:
            tree_path = os.path.join(self.path, "tree", f"tree{count}.tree")

        with open(tree_path, "w+") as file:
            if tree is not None:
                file.write(tree)
        os.chmod(tree_path, 0o666)
