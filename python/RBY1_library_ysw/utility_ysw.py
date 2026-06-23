# =============================================================================
# Utility Function
# =============================================================================
def print_start_title(text: str, width: int = 80, fill: str = "*"):
    msg = f" [{text}] "
    print(msg.center(width, fill) + "\n")
    
def print_end_title(text: str, width: int = 80, fill: str = "*"):
    msg = f" [{text}] "
    print("\n" + msg.center(width, fill) + "\n\n\n")