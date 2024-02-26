import csv

def scale_accelerometer(accel_values):
    return [value / 16384 for value in accel_values]

def scale_magnetometer(mag_values):
    return [value * 0.15 for value in mag_values]

def read_csv(input_file):
    accelerometer_data = []
    magnetometer_data = []
    with open(input_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            accelerometer_values = [float(value) for value in row[:3]]
            magnetometer_values = [float(value) for value in row[3:6]]
            accelerometer_data.append(accelerometer_values)
            magnetometer_data.append(magnetometer_values)
    return accelerometer_data, magnetometer_data

def write_txt(output_file, data):
    with open(output_file, 'w') as file:
        for row in data:
            file.write(' '.join(map(str, row)) + '\n')

def main(input_file, output_accel_file, output_mag_file):
    accelerometer_data, magnetometer_data = read_csv(input_file)
    scaled_accel_data = [scale_accelerometer(row) for row in accelerometer_data]
    scaled_mag_data = [scale_magnetometer(row) for row in magnetometer_data]
    write_txt(output_accel_file, scaled_accel_data)
    write_txt(output_mag_file, scaled_mag_data)

if __name__ == "__main__":
    input_file = "acc_mag_calibration_data.csv"
    output_accel_file = "accelerometer_scaled.txt"
    output_mag_file = "magnetometer_scaled.txt"
    main(input_file, output_accel_file, output_mag_file)
