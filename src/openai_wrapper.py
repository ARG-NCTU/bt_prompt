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
    NEW_OPENAI = parse_version(OPENAI_VERSION) > parse_version("1.0.0")

    def __init__(
        self,
        model="gpt-3.5-turbo-instruct",
        temperature=0,
        max_tokens=500,
        top_p=1,
        presence_penalty=0,
        frequency_penalty=0.2,
    ):
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.top_p = top_p
        self.presence_penalty = presence_penalty
        self.frequency_penalty = frequency_penalty

        self.response = None

    def __get_text_from_response_old(self, response):
        return response['choices'][0]['text']

    def __get_text_from_response_new(self, response):
        return response.choices[0].text

    def get_text_from_response(self, response=None):
        if not response:
            response = self.response

        if self.NEW_OPENAI:
            return self.__get_text_from_response_new(response)
        else:
            return self.__get_text_from_response_old(response)

    def __dump_response_from_response_old(self, response):
        return json.dumps(response, indent=2)

    def __dump_response_from_response_new(self, response):
        return response.model_dump_json(indent=2)

    def dump_response_from_response(self, response=None):
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
        )

    def completion_create(self, prompt):
        if self.NEW_OPENAI:
            self.response = self.__completion_new(prompt)
        else:
            self.response = self.__completion_old(prompt)
        return self.response
