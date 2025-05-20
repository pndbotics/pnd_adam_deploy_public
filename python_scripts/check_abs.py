import json


abs_ip_addr = [
    '10.10.10.60','10.10.10.61','10.10.10.62','10.10.10.63','10.10.10.64','10.10.10.65',
    '10.10.10.80','10.10.10.81','10.10.10.82','10.10.10.83','10.10.10.84','10.10.10.85',
    '10.10.10.100', '10.10.10.101','10.10.10.102']


def validate_abs():
    is_valid = True
    # Read abs.json
    # Check if abs is empty
    try:
        with open("source/abs.json", 'r', encoding='utf-8') as abs_angle_file:
            abs_angle_data = json.load(abs_angle_file)
        # Check if the number of abs is correct
        for ip in abs_ip_addr:
            ip_exists = ip in abs_angle_data and 'motor_rotor_abs_pos' in abs_angle_data[ip] and 'radian' in abs_angle_data[ip]
            if not ip_exists:
                is_valid = False
                print("abs IP: ", ip, " does not exist!")
                return is_valid
    except json.decoder.JSONDecodeError:
        print("abs.json is empty!")
        is_valid = False
        return is_valid
    return is_valid


def main():
    is_valid = validate_abs()
    print(is_valid)


if __name__ == '__main__':
    main()

