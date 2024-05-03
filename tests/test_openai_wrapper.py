import add_path
import pytest

from openai_wrapper import OPENAI_VERSION, OpenAIWrapper, parse_version


@pytest.fixture(scope="module")
def wrapper():
    return OpenAIWrapper()


def test_openai_version_loaded(wrapper):
    assert OPENAI_VERSION != "0.0.0", "OPENAI_VERSION should not be the default placeholder"


def test_openai_wrapper_defaults(wrapper):
    assert wrapper.model == "gpt-3.5-turbo-instruct", "Default model should be set"
    assert wrapper.max_tokens == 500, "Default max_tokens should be set"


def test_completion_create(wrapper):
    response = wrapper.completion_create("test prompt")
    assert response is not None, "Response should not be None"


def test_get_text_from_response(wrapper):
    text = wrapper.get_text_from_response()
    print(f"\ntext={text}\n")
    assert text.strip() != "", "Text should not be empty"


def test_dump_response_from_response(wrapper):
    response = wrapper.dump_response_from_response()
    print(f"\nresponse={response}\n")
    assert response.strip() != "", "Response should not be empty"
