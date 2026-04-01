"""LLM provider abstraction module."""

__all__ = ["LLMProvider", "LLMResponse", "LiteLLMProvider", "OpenAICodexProvider", "AzureOpenAIProvider"]


def __getattr__(name: str):
    if name in {"LLMProvider", "LLMResponse"}:
        from OEA.providers.base import LLMProvider, LLMResponse

        return {"LLMProvider": LLMProvider, "LLMResponse": LLMResponse}[name]
    if name == "LiteLLMProvider":
        from OEA.providers.litellm_provider import LiteLLMProvider

        return LiteLLMProvider
    if name == "OpenAICodexProvider":
        from OEA.providers.openai_codex_provider import OpenAICodexProvider

        return OpenAICodexProvider
    if name == "AzureOpenAIProvider":
        from OEA.providers.azure_openai_provider import AzureOpenAIProvider

        return AzureOpenAIProvider
    raise AttributeError(name)
