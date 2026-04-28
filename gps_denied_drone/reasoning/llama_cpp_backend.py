"""llama.cpp backend for the on-device SLM advisor.

Provides a backend object with a ``.generate(prompt, max_tokens) -> str``
method that ``GeminiNanoClient`` consumes. Loads a GGUF model
(default: Gemma 3 1B Q4_K_M) via llama_cpp_python.

Lazy import: if ``llama_cpp`` is not installed, ``LlamaCppBackend(...)``
raises ImportError on construction so callers can fall back to
``rule_tree`` cleanly. CI does not depend on llama_cpp; the production
Pi 5 build does.

Install on Pi 5:
    pip install llama-cpp-python

Recommended GGUF for the advisor role (~1 GB RAM, 5-10 tok/s on
Cortex-A76):
    https://huggingface.co/google/gemma-3-1b-it-qat-q4_0-gguf

Quick test:
    from gps_denied_drone.reasoning.llama_cpp_backend import LlamaCppBackend
    from gps_denied_drone.reasoning.gemini_nano import GeminiNanoClient
    backend = LlamaCppBackend("/path/to/gemma-3-1b.q4_k_m.gguf")
    client = GeminiNanoClient(backend=backend)
    print(client.decide({"slam_tracking_ok": True, "battery_pct": 80}))
"""

from __future__ import annotations
import os
from typing import Any


class LlamaCppBackend:
    """Thin generate-once wrapper. The advisor schema is JSON only,
    so we use llama_cpp's grammar-constrained decoding when available
    to keep outputs parseable."""

    def __init__(self,
                 model_path: str,
                 n_ctx: int = 2048,
                 n_threads: int | None = None,
                 temperature: float = 0.0,
                 use_json_grammar: bool = True):
        try:
            from llama_cpp import Llama  # type: ignore[import-not-found]
        except Exception as e:                       # pragma: no cover
            raise ImportError(
                "llama_cpp_python is not installed. "
                "Run `pip install llama-cpp-python` on the deployment "
                "target, or call create_backend(name='stub') to fall "
                "back to the rule-tree advisor.") from e

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"GGUF not found: {model_path}")

        self.temperature = float(temperature)
        self.use_json_grammar = bool(use_json_grammar)
        self._llm = Llama(
            model_path=model_path,
            n_ctx=int(n_ctx),
            n_threads=n_threads,
            verbose=False,
        )
        self._grammar = None
        if self.use_json_grammar:
            self._grammar = _try_make_json_grammar()

    def generate(self, prompt: str, max_tokens: int = 192) -> str:
        out = self._llm(
            prompt,
            max_tokens=int(max_tokens),
            temperature=self.temperature,
            stop=["\nOBSERVATION:", "\n\n"],
            grammar=self._grammar,
        )
        return out["choices"][0]["text"]


def _try_make_json_grammar():
    """Build a minimal JSON-object grammar so the model can't ramble.
    Returns None on any error -- the GeminiNanoClient parser already
    tolerates slop."""
    try:
        from llama_cpp.llama_grammar import LlamaGrammar  # type: ignore
        return LlamaGrammar.from_string(_JSON_GRAMMAR)
    except Exception:                            # pragma: no cover
        return None


_JSON_GRAMMAR = r"""
root   ::= object
object ::= "{" ws (pair (ws "," ws pair)*)? ws "}"
pair   ::= string ws ":" ws value
value  ::= string | number | object | array | "true" | "false" | "null"
array  ::= "[" ws (value (ws "," ws value)*)? ws "]"
string ::= "\"" ([^"\\] | "\\" .)* "\""
number ::= "-"? ("0" | [1-9][0-9]*) ("." [0-9]+)? ([eE] [-+]? [0-9]+)?
ws     ::= [ \t\n]*
"""


def create_backend(name: str = "stub", **kwargs: Any):
    """Factory used by ROS 2 launch and the runner.

    name:
      "stub"                    -> None (caller falls back to rule tree)
      "llama_cpp_gemma3_1b"     -> LlamaCppBackend with model path from
                                   $GEMMA3_GGUF or kwargs['model_path']
      "llama_cpp"               -> LlamaCppBackend with explicit
                                   kwargs['model_path']
    """
    if name == "stub" or not name:
        return None
    if name == "llama_cpp_gemma3_1b":
        path = kwargs.pop("model_path", None) or os.environ.get("GEMMA3_GGUF")
        if not path:
            raise ValueError(
                "GEMMA3_GGUF env var or kwargs['model_path'] required "
                "for backend='llama_cpp_gemma3_1b'.")
        return LlamaCppBackend(path, **kwargs)
    if name == "llama_cpp":
        path = kwargs.pop("model_path")
        return LlamaCppBackend(path, **kwargs)
    raise ValueError(f"unknown SLM backend: {name!r}")
