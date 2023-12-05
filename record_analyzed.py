def parse_line(line):
    """Parse each line and return a dictionary containing all relevant data"""
    parts = line.split(' - ')
    data = {}
    for part in parts[1:]:
        if ':' in part:
            key, value = part.split(':')
            if key in ['distance', 'nearby_car']:
                data[key] = float(value)
            elif key == 'crashed':
                data[key] = True if value.strip() == 'True' else False
            elif key == 'valid_frames/num_frames':
                valid_frames, num_frames = value.split('/')
                data['valid_frames'] = int(valid_frames)
                data['num_frames'] = int(num_frames)
    return data

def main(filename):
    total_rounds = 0
    total_distance = 0
    total_nearby_car = 0
    total_crashed = 0
    total_valid_frames = 0
    total_num_frames = 0
    sum_frame_ratios = 0

    with open(filename, 'r') as file:
        for line in file:
            data = parse_line(line)
            if 'distance' in data:
                total_distance += data['distance']
                total_rounds += 1
            if 'nearby_car' in data:
                total_nearby_car += data['nearby_car']
            if data.get('crashed', False):
                total_crashed += 1
            if 'valid_frames' in data and 'num_frames' in data and data['num_frames'] > 0:
                total_valid_frames += data['valid_frames']
                total_num_frames += data['num_frames']
                sum_frame_ratios += data['valid_frames'] / data['num_frames']

    average_distance = total_distance / total_rounds if total_rounds > 0 else 0
    average_nearby_car = total_nearby_car / total_rounds if total_rounds > 0 else 0
    average_frame_ratio = sum_frame_ratios / total_rounds if total_rounds > 0 else 0
    overall_frame_ratio = total_valid_frames / total_num_frames if total_num_frames > 0 else 0

    results = [
        f"Total Rounds: {total_rounds}",
        f"Distance - Total: {total_distance}, Average: {average_distance}",
        f"Nearby Car - Total: {total_nearby_car}, Average: {average_nearby_car}",
        f"Total Crashed: {total_crashed}",
        f"Average Frame Ratio: {average_frame_ratio}",
        f"Overall Frame Ratio: {overall_frame_ratio}"
    ]

    with open('result.log', 'w') as result_file:
        for line in results:
            print(line)
            result_file.write(line + '\n')

if __name__ == "__main__":
    main('record.log')
