import re
import pandas as pd
import os

def extract_last_cost(file_path):
    """
    Extracts the last cost value for each input VRPTW file from a given text file.

    Args:
        file_path (str): The path to the input text file (e.g., D-1.txt).

    Returns:
        dict: A dictionary where keys are VRPTW input filenames (e.g., 'E-n76-k8.vrp')
              and values are their corresponding last cost values.
    """
    costs_data = {}
    try:
        with open(file_path, 'r') as f:
            for line in f:
                # Regex for D-x.txt files: captures filename and three cost values (floats)
                # Example: ../inputs/E-n76-k8.vrp Cost 1044.54 888.867 824.947
                match_dx = re.search(r'../inputs/(.*?\.vrp)\s+Cost\s+\d+\.?\d*\s+\d+\.?\d*\s+(\d+\.?\d*)', line)
                
                # Regex for rajeshParMDSResults.txt: captures filename and three cost values (integers)
                # Example: /usrscratch/.../E-n76-k8.vrp Cost 1017 906 823
                match_rajesh = re.search(r'/usrscratch/.*/inputs/(.*?\.vrp)\s+Cost\s+\d+\s+\d+\s+(\d+)', line)

                if match_dx:
                    input_file = match_dx.group(1)
                    last_cost = float(match_dx.group(2))
                    costs_data[input_file] = last_cost
                elif match_rajesh:
                    input_file = match_rajesh.group(1)
                    last_cost = int(match_rajesh.group(2)) # Store as int, can convert to float later if needed
                    costs_data[input_file] = float(last_cost) # Convert to float for consistency
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
    except Exception as e:
        print(f"An error occurred while processing {file_path}: {e}")
    return costs_data

def main():
    # Define the paths to your local input files
    # IMPORTANT: Ensure these paths are correct relative to where you run the script,
    # or provide absolute paths.
    file_paths = {
        "D-1": "D-1.txt",
        "D-3": "D-3.txt",
        "D-5": "D-5.txt",
        "D-7": "D-7.txt",
        "D-10": "D-10.txt",
        "rajeshParMDSResults": "rajeshParMDSResults.txt"
    }

    all_data = {} # To store all extracted costs, grouped by input VRPTW file

    # Process each file
    for col_name, file_name in file_paths.items():
        print(f"Processing {file_name}...")
        extracted_costs = extract_last_cost(file_name)
        
        # Merge extracted costs into the main all_data dictionary
        # Initialize the nested dictionary if the input_file is new
        for input_file, cost in extracted_costs.items():
            if input_file not in all_data:
                all_data[input_file] = {}
            all_data[input_file][col_name] = cost

    # Convert the collected data into a pandas DataFrame for easy CSV export
    # First, create a list of dictionaries, one for each row
    rows_list = []
    
    # Get all unique input files and sort them for consistent output order
    all_input_files = sorted(all_data.keys())

    for input_file in all_input_files:
        row_dict = {"Input File": input_file}
        for col_name in file_paths.keys():
            # Get the cost for this input_file and column, or use None/empty string if not found
            row_dict[col_name] = all_data[input_file].get(col_name)
        rows_list.append(row_dict)

    df = pd.DataFrame(rows_list)

    # Define the output CSV filename
    output_csv_file = "cost_comparison_output.csv"

    # Export the DataFrame to a CSV file
    df.to_csv(output_csv_file, index=False)

    print(f"\nCSV file '{output_csv_file}' has been generated successfully!")
    print(f"It contains {len(df)} rows and {len(df.columns)} columns.")

if __name__ == "__main__":
    main()

