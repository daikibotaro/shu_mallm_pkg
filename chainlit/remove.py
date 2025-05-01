def remove_until_target(input_str, target):
    index = str(input_str).find(target)
    if index != -1:
        output_str = input_str[index + len(target):]
        return output_str
    else:
        return input_str


def remove_after_target(input_str, target):
    index = str(input_str).find(target)
    if index != -1:
        output_str = input_str[:index]
        return output_str
    else:
        return input_str
