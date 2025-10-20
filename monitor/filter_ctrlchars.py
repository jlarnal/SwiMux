from platformio.public import DeviceMonitorFilterBase

class Ctrlchars(DeviceMonitorFilterBase):
    """
    A serial monitor filter that replaces specific ASCII control characters
    with their hexadecimal escape sequence representation. It skips characters
    in the range 0x08 to 0x13 to preserve terminal formatting.
    """
    NAME = "ctrlchars"

    def rx(self, text):
        """Called with received text."""
        processed_chars = []
        for char in text:
            code = ord(char)
            # Only convert characters in the ranges [0-7] and [14-31].
            if (code <= 7) or (14 <= code <= 31) or (code > 127):
                processed_chars.append(f"[0x{code:02x}]")
            else:
                processed_chars.append(char)
        return "".join(processed_chars)

    def tx(self, text):
        """Called with text sent to the serial port."""
        # Pass sent text through without modification.
        return text