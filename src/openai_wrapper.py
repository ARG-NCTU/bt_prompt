import json
import os

from pkg_resources import parse_version

# Check openai version
OPENAI_VERSION = "0.0.0"
try:
    import openai
    OPENAI_VERSION = openai.__version__
except AttributeError:
    import openai
    OPENAI_VERSION = openai.version.VERSION
except ImportError:
    raise ImportError("Please install openai package. pip install openai")

# Check if the openai version is greater than 1.0.0
if parse_version(OPENAI_VERSION) > parse_version("1.0.0"):
    from openai import OpenAI
    client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
else:
    openai.api_key = os.environ["OPENAI_API_KEY"]


class OpenAIWrapper:
    """
    A wrapper class for interacting with the OpenAI API.

    Args:
        model (str): The name of the OpenAI model to use. Default is "gpt-3.5-turbo-instruct".
        temperature (float): Controls the randomness of the output. Higher values make the output more random. Default is 0.
        max_tokens (int): The maximum number of tokens in the generated response. Default is 500.
        top_p (float): Controls the diversity of the output. Lower values make the output more focused. Default is 1.
        presence_penalty (float): Controls the model's preference for including or avoiding certain phrases. Default is 0.
        frequency_penalty (float): Controls the model's preference for using frequent or rare words. Default is 0.2.
        **kwargs: Additional keyword arguments to be passed to the OpenAI API.

    Attributes:
        model (str): The name of the OpenAI model being used.
        temperature (float): The temperature value for generating responses.
        max_tokens (int): The maximum number of tokens in the generated response.
        top_p (float): The top-p value for generating responses.
        presence_penalty (float): The presence penalty value for generating responses.
        frequency_penalty (float): The frequency penalty value for generating responses.
        kwargs (dict): Additional keyword arguments passed to the OpenAI API.
        response (object): The response object returned by the OpenAI API.

    Methods:
        get_text_from_response(response=None): Extracts the text from the response object.
        dump_response_from_response(response=None): Dumps the response object as a JSON string.
        completion_create(prompt): Creates a completion using the specified prompt.

    """

    NEW_OPENAI = parse_version(OPENAI_VERSION) > parse_version("1.0.0")

    def __init__(
        self,
        model="gpt-3.5-turbo-instruct",
        temperature=0,
        max_tokens=500,
        top_p=1,
        presence_penalty=0,
        frequency_penalty=0.2,
        **kwargs,
    ):
        """
        Initializes an instance of the OpenAIWrapper class.

        Args:
            model (str): The name of the GPT model to use. Defaults to "gpt-3.5-turbo-instruct".
            temperature (float): Controls the randomness of the generated text. Defaults to 0.
            max_tokens (int): The maximum number of tokens in the generated text. Defaults to 500.
            top_p (float): Controls the diversity of the generated text. Defaults to 1.
            presence_penalty (float): Controls the model's preference for including or avoiding certain phrases. Defaults to 0.
            frequency_penalty (float): Controls the model's preference for using or avoiding repeated phrases. Defaults to 0.2.
            **kwargs: Additional keyword arguments to be passed to the OpenAI API.

        Returns:
            None
        """
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.top_p = top_p
        self.presence_penalty = presence_penalty
        self.frequency_penalty = frequency_penalty
        self.kwargs = kwargs

        self.response = None

    def __get_text_from_response_old(self, response):
        return response['choices'][0]['text']

    def __get_text_from_response_new(self, response):
        return response.choices[0].text
    
    def __get_text_from_response_chat(self, response):
        return response.choices[0].message.content

    def get_text_from_response(self, response=None):
        """
        Extracts the text from the response object.

        Args:
            response (object): The response object. If not provided, the stored response object will be used.

        Returns:
            str: The extracted text from the response.

        """
        if not response:
            response = self.response

        if self.NEW_OPENAI:
            if isinstance(response, openai.types.chat.chat_completion.ChatCompletion):
                return self.__get_text_from_response_chat(response)
            
            return self.__get_text_from_response_new(response)
        else:
            return self.__get_text_from_response_old(response)

    def __dump_response_from_response_old(self, response):
        return json.dumps(response, indent=2)

    def __dump_response_from_response_new(self, response):
        return response.model_dump_json(indent=2)

    def dump_response_from_response(self, response=None):
        """
        Dumps the response object as a JSON string.

        Args:
            response (object): The response object. If not provided, the stored response object will be used.

        Returns:
            str: The response object dumped as a JSON string.

        """
        if not response:
            response = self.response

        if self.NEW_OPENAI:
            return self.__dump_response_from_response_new(response)
        else:
            return self.__dump_response_from_response_old(response)

    def __completion_old(self, prompt):
        return openai.Completion.create(
            model=self.model,
            prompt=prompt,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            top_p=self.top_p,
            presence_penalty=self.presence_penalty,
            frequency_penalty=self.frequency_penalty,
            **self.kwargs,
        )

    def __completion_new(self, prompt):
        return client.completions.create(
            model=self.model,
            prompt=prompt,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            top_p=self.top_p,
            presence_penalty=self.presence_penalty,
            frequency_penalty=self.frequency_penalty,
            **self.kwargs,
        )

    def completion_create(self, prompt):
        """
        Creates a completion using the specified prompt.

        Args:
            prompt (str): The prompt to generate the completion.

        Returns:
            object: The response object returned by the OpenAI API.

        """
        if self.NEW_OPENAI:
            self.response = self.__completion_new(prompt)
        else:
            self.response = self.__completion_old(prompt)
        return self.response

    def chat_completion(self, prompt):
        """
        Creates a completion using the specified prompt.

        Args:
            prompt (str): The prompt to generate the completion.

        Returns:
            object: The response object returned by the OpenAI API.

        """
        return client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            top_p=self.top_p,
            presence_penalty=self.presence_penalty,
            frequency_penalty=self.frequency_penalty,
            **self.kwargs,
        )
