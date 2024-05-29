import json
from langchain_core.prompts import PromptTemplate
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_openai import OpenAI
from langchain_core.output_parsers import JsonOutputParser


class Joke(BaseModel):
    setup: str = Field(description="question to set up a joke")
    punchline: str = Field(description="answer to resolve the joke")

# And a query intented to prompt a language model to populate the data structure.
joke_query = "Tell me a joke."


model = OpenAI(model_name="gpt-3.5-turbo-instruct", openai_api_key="YOUR_API_KEY_HERE")

# Set up a parser + inject instructions into the prompt template.
parser = JsonOutputParser(pydantic_object=Joke)
print(parser.get_format_instructions())
print("=====================================================")
prompt = PromptTemplate(
    template="Answer the user query.\n{format_instructions}\n{query}\n",
    input_variables=["query"],
    partial_variables={"format_instructions": parser.get_format_instructions()},
)

chain = prompt | model | parser

respond = chain.invoke({"query": joke_query})
print(respond)