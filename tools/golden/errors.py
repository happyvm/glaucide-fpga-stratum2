"""Common exception for malformed input."""


class ParseError(ValueError):
    """Raised by every parse() on malformed input (bad checksum, bad length...)."""
