"""
INS data .csv files reader and elaborate utility
==============================================================================
Author:        Luca Erviati
Date:          2025-01-13
Version:       1.0

CSVTool is a utility class for processing and extracting data from CSV files containing INS PVA and INS CORRIMU data.

    Attributes:
        inspva_path (Path): Path to the directory containing INS PVA CSV files.
        corrimu_path (Path): Path to the directory containing INS CORRIMU CSV files.
        csv_files_inspva (list): List of INS PVA CSV file paths.
        csv_files_corrimu (list): List of INS CORRIMU CSV file paths.
        csv_rows_inspva (list): List of rows from INS PVA CSV files.
        csv_rows_corrimu (list): List of rows from INS CORRIMU CSV files.
        csv_data_inspva (list): Extracted INS PVA data (north, east, up velocity).
        csv_data_corrimu (list): Extracted INS CORRIMU data (lateral, longitudinal, vertical acceleration).

    Methods:
        list_csv_files():
            Lists and sorts all CSV files in the specified directories for INS PVA and INS CORRIMU data.

        read_csv_files(csv_files, csv_rows, csv_data, data_indices):
            Reads the specified CSV files, extracts the rows and specific data columns based on provided indices.

        process_csv_files():
            Processes the CSV files by listing them and reading their contents to extract relevant data.

        print_csv_rows():
            Prints the rows of the INS PVA and INS CORRIMU CSV files, as well as the extracted data.
    
==============================================================================
"""

import csv
from pathlib import Path
from geopy.distance import geodesic

class CSVTool:
    def __init__(self):

        inspva_path = '/Users/luca/Downloads/VisualOdometry_Erviati_HITACHI/3_fire_site_3/novatel_oem7_inspva'
        corrimu_path = '/Users/luca/Downloads/VisualOdometry_Erviati_HITACHI/3_fire_site_3/novatel_oem7_corrimu'
        insstdev_path = '/Users/luca/Downloads/VisualOdometry_Erviati_HITACHI/3_fire_site_3/novatel_oem7_insstdev'


        self.inspva_path = Path(inspva_path)        # Path to the INS PVA data
        self.corrimu_path = Path(corrimu_path)      # Path to the INS CORRIMU data
        self.insstdev_path = Path(insstdev_path)    # Path to the INS INNSTDEV data

        self.csv_files_inspva = []                  # List of INS PVA .csv files
        self.csv_files_corrimu = []                 # List of INS CORRIMU .csv files
        self.csv_files_insstdev = []                # List of INS INNSTDEV .csv files

        self.csv_rows_inspva = []                   # List of INS PVA .csv rows
        self.csv_rows_corrimu = []                  # List of INS CORRIMU .csv rows
        self.csv_rows_insstdev = []                 # List of INS INNSTDEV .csv rows

        self.ground_truth_data = []                 # List of ground truth data
        self.ground_truth_speeds = []               # List of ground truth speeds



    def list_csv_files(self):
        self.csv_files_inspva = [x for x in self.inspva_path.iterdir() if x.is_file()]
        self.csv_files_corrimu = [x for x in self.corrimu_path.iterdir() if x.is_file()]
        self.csv_files_insstdev = [x for x in self.insstdev_path.iterdir() if x.is_file()]

        self.csv_files_inspva.sort()
        self.csv_files_corrimu.sort()
        self.csv_files_insstdev.sort()
    

    def read_csv_files(self, csv_files, csv_rows):
        for csv_file in csv_files:
            with open(csv_file, newline='') as f:
                reader = csv.reader(f)
                next(reader)  # Skip header
                for row in reader:
                    csv_rows.append(row)

    def get_corrimu_data(self):
        self.read_csv_files(self.csv_files_corrimu, self.csv_rows_corrimu)
        return self.csv_rows_corrimu

    def get_inspva_data(self):
        self.read_csv_files(self.csv_files_inspva, self.csv_rows_inspva)
        return self.csv_rows_inspva

    def get_insstdev_data(self):
        self.read_csv_files(self.csv_files_insstdev, self.csv_rows_insstdev)
        return self.csv_rows_insstdev

    def get_ground_truth_data(self, inspva_data, corrimu_data):
        #ground_truth_data = []
        for inspva_row, corrimu_row in zip(inspva_data, corrimu_data):
            data = {
                'latitude': float(inspva_row[2]),
                'longitude': float(inspva_row[3]),
                'north_velocity': float(inspva_row[5]),
                'east_velocity': float(inspva_row[6]),
                'up_velocity': float(inspva_row[7]),
                'longitudinal_acc': float(corrimu_row[7]),
                'lateral_acc': float(corrimu_row[8]),
                'vertical_acc': float(corrimu_row[9])
            }
            self.ground_truth_data.append(data)
        return self.ground_truth_data

    def compute_ground_truth_speeds(self, ground_truth_data):
        #ground_truth_data = self.get_ground_truth_data()
        #speeds = []

        for i in range(1, len(ground_truth_data)):
            prev_data = ground_truth_data[i - 1]
            curr_data = ground_truth_data[i]
            
            # Calculate distance between consecutive GPS points
            prev_coords = (prev_data['latitude'], prev_data['longitude'])
            curr_coords = (curr_data['latitude'], curr_data['longitude'])
            distance = geodesic(prev_coords, curr_coords).meters
            
            # Calculate time difference
            time_diff = 0.1  # Assuming 100ms sampling time
            
            # Calculate speeds
            speed_m_s = distance / time_diff
            speed_kmh = speed_m_s * 3.6
            self.ground_truth_speeds.append(speed_kmh)
            
            # Add speeds to the current data
            curr_data['ground_truth_speed'] = speed_kmh
        
        return self.ground_truth_speeds

def main():
    csv_tool = CSVTool()
    csv_tool.list_csv_files()

    print("CSV Files INSPVA:")
    print(csv_tool.csv_files_inspva)
    print("CSV Files CORRIMU:")
    print(csv_tool.csv_files_corrimu)
    print("CSV Files INSSTDEV:")
    print(csv_tool.csv_files_insstdev)

    corrimu_data = csv_tool.get_corrimu_data()
    inspva_data = csv_tool.get_inspva_data()
    insstdev_data = csv_tool.get_insstdev_data()
    ground_truth_data = csv_tool.get_ground_truth_data(inspva_data, corrimu_data)

    print("Ground Truth Data:")
    for data in ground_truth_data:
        print(data)

    print("CORRIMU Data: ")
    for data in corrimu_data:
        print(data)

    print("INSPVA Data: ")
    for data in inspva_data:
        print(data)

    print("INSSTDEV Data: ")
    for data in insstdev_data:
        print(data)

    ground_truth_speed = csv_tool.compute_ground_truth_speeds(ground_truth_data)
    print("Ground Truth Data with Speeds:")
    for data in ground_truth_speed:
        print(data)

if __name__ == "__main__":
    main()