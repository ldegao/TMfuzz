import re

def process_file(input_file, output_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    with open(output_file, 'w') as file:
        for line in lines:
            # Look for the pattern "valid_time/time: [number]/[number]"
            match = re.search(r'valid_time/time: (\d+)/(\d+)', line)
            if match:
                # Divide the numbers by 5
                num1 = int(match.group(1)) / 5
                num2 = int(match.group(2)) / 5
                # Replace the original numbers with the new ones
                new_line = re.sub(r'valid_time/time: \d+/\d+', f'valid_time/time: {num1}/{num2}', line)
                file.write(new_line)
            else:
                file.write(line)

# Replace 'record.log' with the path to your file
process_file('record.log', 'processed_record.log')
