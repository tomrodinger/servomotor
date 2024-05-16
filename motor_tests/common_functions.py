
def print_test_description(test_description):
    print("\n****************************************************************************************************************************")
    print(test_description)
    print("****************************************************************************************************************************\n")


def add_standard_option_to_parser(parser):
    # Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
    # and it also takes a mandatory firmware file name
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    return parser.parse_args()


def get_alias_list(parsed_response):
    alias_list = []
    if isinstance(parsed_response, list):
        if all(isinstance(i, int) for i in parsed_response) and len(parsed_response) == 3:
            alias_list.append(parsed_response[1])
        elif all(isinstance(sublist, list) and len(sublist) == 3 and all(isinstance(i, int) for i in sublist) for sublist in parsed_response):
            alias_list = [sublist[1] for sublist in parsed_response]
    if len(alias_list) == 0:
        print("ERROR: No devices were detected")
        print("*** FAILED ***")
        exit(1)
    return alias_list


def get_unique_id_list(parsed_response):
    unique_id_list = []
    if isinstance(parsed_response, list):
        if all(isinstance(i, int) for i in parsed_response) and len(parsed_response) == 3:
            unique_id_list.append(parsed_response[0])
        elif all(isinstance(sublist, list) and len(sublist) == 3 and all(isinstance(i, int) for i in sublist) for sublist in parsed_response):
            unique_id_list = [sublist[0] for sublist in parsed_response]
    if len(unique_id_list) == 0:
        print("ERROR: No devices were detected")
        print("*** FAILED ***")
        exit(1)
    return unique_id_list


def get_human_readable_alias(alias):
    #check if this is a human readable ASCII character
    if 32 <= alias <= 126:
        ascii = chr(alias)
        return f"{ascii} (ASCII) {alias} (decimal)"
    else:
        return f"{alias} (decimal)"


def print_alias_list(alias_list):
    for alias in alias_list:
        s = get_human_readable_alias(alias)
        print(f"Device detected with alias: {s}")


# The human readable unique ID is representaed as an 8 character hexadecimal number
def get_human_readable_unique_id(unique_id):
    return f"{unique_id:08X}"


def test_failed():
    print("*** FAILED ***")
    exit(1)


def test_passed():
    print("*** PASSED ***")