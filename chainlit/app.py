from rich import print
from openai import AsyncOpenAI
import chainlit as cl
client = AsyncOpenAI()


# Instrument the OpenAI client
cl.instrument_openai()

settings = {
    "model": "gpt-3.5-turbo",
    "temperature": 0,
    # ... more settings
}


@cl.on_message
async def on_message(message: cl.Message):
    response = await client.chat.completions.create(
        messages=[
            {
                "content": "You are a helpful bot",
                "role": "system"
            },
            {
                "content": message.content,
                "role": "user"
            }
        ],
        **settings
    )
    completion_tokens, prompt_usage, total_usage = response.usage.completion_tokens, response.usage.prompt_tokens, response.usage.total_tokens
    print(
        f"\ncompletion_tokens: {completion_tokens}  prompt_tokens: {prompt_usage}  total usage: {total_usage}")
    await cl.Message(content=response.choices[0].message.content).send()
