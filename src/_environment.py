import os
import sys
import signal

_sigint_count = 0


# Suppress "qt5ct: using qt5ct plugin" message
def setup_environment(cleanup_callback):
    os.environ["QT_LOGGING_RULES"] = "qt5ct.debug=false"

    def handle_sigint(sig, frame):
        global _sigint_count
        _sigint_count += 1
        if _sigint_count >= 2:
            print("\n\nExiting...")
            cleanup_callback()
            sys.exit(0)
        else:
            print("\n\nPress Ctrl+C again to confirm interrupt")

    signal.signal(signal.SIGINT, handle_sigint)
