"""Agent core module."""

__all__ = ["AgentLoop", "ContextBuilder", "MemoryStore", "SkillsLoader"]


def __getattr__(name: str):
    if name == "AgentLoop":
        from OEA.agent.loop import AgentLoop

        return AgentLoop
    if name == "ContextBuilder":
        from OEA.agent.context import ContextBuilder

        return ContextBuilder
    if name == "MemoryStore":
        from OEA.agent.memory import MemoryStore

        return MemoryStore
    if name == "SkillsLoader":
        from OEA.agent.skills import SkillsLoader

        return SkillsLoader
    raise AttributeError(name)
