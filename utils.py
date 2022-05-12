def flatten(input_list):
    """Recursively iterate through values in nested lists."""
    for item in input_list:
        if isinstance(item, list): # Use what ever nesting condition you need here
            for child_item in flatten(item):
                yield child_item
        else:
            yield item

def has_items(seq):
    """Checks if an iterator has any items."""
    return any(1 for _ in seq)